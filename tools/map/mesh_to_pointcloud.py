#!/usr/bin/env python3
"""把三角网格模型 (mesh) 转换为表面点云 (point cloud)。

仅依赖 numpy，读取 binary_little_endian 的 PLY 网格，在三角面上做
按面积加权的均匀采样，输出只包含表面点(带法线)的点云 PLY。

用法:
    python3 mesh_to_pointcloud.py input.ply output.ply [--num-points N | --density D] [--vertices-only]

    --num-points N   采样总点数 (默认 500000)
    --density D      每平方米采样点数, 指定后覆盖 --num-points
    --vertices-only  不采样, 直接把网格顶点导出为点云
"""
import argparse
import struct
import sys
import numpy as np


def read_ply_mesh(path):
    with open(path, "rb") as f:
        data = f.read()

    # ---- 解析 header (ASCII) ----
    end_tag = b"end_header\n"
    hidx = data.find(end_tag)
    if hidx < 0:
        sys.exit("找不到 end_header")
    header = data[:hidx].decode("ascii", "replace").splitlines()
    body = data[hidx + len(end_tag):]

    if not any("binary_little_endian" in h for h in header):
        sys.exit("仅支持 binary_little_endian 格式的 PLY")

    # 每个 element 的属性列表
    elements = []  # (name, count, [(prop_type, prop_name) or ('list', count_type, item_type, name)])
    cur = None
    for line in header:
        toks = line.split()
        if not toks:
            continue
        if toks[0] == "element":
            cur = {"name": toks[1], "count": int(toks[2]), "props": []}
            elements.append(cur)
        elif toks[0] == "property" and cur is not None:
            if toks[1] == "list":
                cur["props"].append(("list", toks[2], toks[3], toks[4]))
            else:
                cur["props"].append((toks[1], toks[2]))

    TYPE = {
        "char": ("b", 1), "uchar": ("B", 1), "int8": ("b", 1), "uint8": ("B", 1),
        "short": ("h", 2), "ushort": ("H", 2), "int16": ("h", 2), "uint16": ("H", 2),
        "int": ("i", 4), "uint": ("I", 4), "int32": ("i", 4), "uint32": ("I", 4),
        "float": ("f", 4), "float32": ("f", 4), "double": ("d", 8), "float64": ("d", 8),
    }

    verts = None
    faces = []
    off = 0
    for el in elements:
        if el["name"] == "vertex":
            # 假定所有顶点属性都是标量, 固定步长 -> 用 numpy 结构化读取
            fmt_fields = []
            names = []
            for p in el["props"]:
                if p[0] == "list":
                    sys.exit("vertex 元素含 list 属性, 不支持")
                fc, _ = TYPE[p[0]]
                fmt_fields.append(("<" + fc))
                names.append(p[1])
            dt = np.dtype([(n, f) for n, f in zip(names, fmt_fields)])
            n = el["count"]
            block = np.frombuffer(body, dtype=dt, count=n, offset=off)
            off += dt.itemsize * n
            xyz = np.stack([block["x"], block["y"], block["z"]], axis=1).astype(np.float64)
            if all(k in block.dtype.names for k in ("nx", "ny", "nz")):
                nrm = np.stack([block["nx"], block["ny"], block["nz"]], axis=1).astype(np.float64)
            else:
                nrm = None
            verts = (xyz, nrm)
        elif el["name"] == "face":
            # face 含 list 属性, 变长 -> 逐个解析
            cnt_c, cnt_sz = TYPE[el["props"][0][1]]
            item_c, item_sz = TYPE[el["props"][0][2]]
            # face 后可能还有标量属性(如 flags)
            trailing = el["props"][1:]
            trail_fmt = "<" + "".join(TYPE[p[0]][0] for p in trailing)
            trail_sz = sum(TYPE[p[0]][1] for p in trailing)
            for _ in range(el["count"]):
                (k,) = struct.unpack_from("<" + cnt_c, body, off)
                off += cnt_sz
                idx = struct.unpack_from("<" + item_c * k, body, off)
                off += item_sz * k
                off += trail_sz  # 跳过 flags 等
                faces.append(idx)
        else:
            sys.exit(f"未处理的 element: {el['name']}")

    return verts, faces


def triangulate(faces):
    """把多边形面拆成三角形 (fan)。"""
    tris = []
    for fidx in faces:
        if len(fidx) < 3:
            continue
        for i in range(1, len(fidx) - 1):
            tris.append((fidx[0], fidx[i], fidx[i + 1]))
    return np.asarray(tris, dtype=np.int64)


def sample_surface(xyz, tris, num_points, rng):
    v0 = xyz[tris[:, 0]]
    v1 = xyz[tris[:, 1]]
    v2 = xyz[tris[:, 2]]
    # 三角面面积
    areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    total = areas.sum()
    if total <= 0:
        sys.exit("网格面积为 0, 无法采样")
    probs = areas / total
    # 按面积选面
    chosen = rng.choice(len(tris), size=num_points, p=probs)
    # 三角形内均匀重心采样
    r1 = np.sqrt(rng.random(num_points))
    r2 = rng.random(num_points)
    a = 1.0 - r1
    b = r1 * (1.0 - r2)
    c = r1 * r2
    pts = (a[:, None] * v0[chosen]
           + b[:, None] * v1[chosen]
           + c[:, None] * v2[chosen])
    # 面法线
    fn = np.cross(v1[chosen] - v0[chosen], v2[chosen] - v0[chosen])
    ln = np.linalg.norm(fn, axis=1, keepdims=True)
    ln[ln == 0] = 1.0
    normals = fn / ln
    return pts, normals


def write_ply_pointcloud(path, xyz, normals=None):
    n = len(xyz)
    has_n = normals is not None
    with open(path, "wb") as f:
        head = ["ply", "format binary_little_endian 1.0",
                f"element vertex {n}",
                "property float x", "property float y", "property float z"]
        if has_n:
            head += ["property float nx", "property float ny", "property float nz"]
        head += ["end_header\n"]
        f.write(("\n".join(head)).encode("ascii"))
        if has_n:
            arr = np.hstack([xyz, normals]).astype("<f4")
        else:
            arr = xyz.astype("<f4")
        f.write(arr.tobytes())


def write_pcd_pointcloud(path, xyz, normals=None):
    n = len(xyz)
    has_n = normals is not None
    if has_n:
        fields = "x y z normal_x normal_y normal_z"
        sizes = "4 4 4 4 4 4"
        types = "F F F F F F"
        counts = "1 1 1 1 1 1"
        arr = np.hstack([xyz, normals]).astype("<f4")
    else:
        fields = "x y z"
        sizes = "4 4 4"
        types = "F F F"
        counts = "1 1 1"
        arr = xyz.astype("<f4")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {fields}\n"
        f"SIZE {sizes}\n"
        f"TYPE {types}\n"
        f"COUNT {counts}\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(arr.tobytes())


def write_pointcloud(path, xyz, normals=None):
    """按扩展名选择 PLY 或 PCD。"""
    if path.lower().endswith(".pcd"):
        write_pcd_pointcloud(path, xyz, normals)
    else:
        write_ply_pointcloud(path, xyz, normals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--num-points", type=int, default=500000)
    ap.add_argument("--density", type=float, default=None,
                    help="每平方米点数, 指定后覆盖 --num-points")
    ap.add_argument("--vertices-only", action="store_true")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="坐标缩放系数, 毫米转米用 0.001")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    (xyz, vnrm), faces = read_ply_mesh(args.input)
    if args.scale != 1.0:
        xyz = xyz * args.scale
        print(f"坐标缩放: x{args.scale}")
    mn, mx = xyz.min(0), xyz.max(0)
    print(f"顶点数: {len(xyz)}  面数: {len(faces)}")
    print(f"包围盒 min: {mn.round(3)}  max: {mx.round(3)}  尺寸: {(mx-mn).round(3)}")

    if args.vertices_only:
        write_pointcloud(args.output, xyz, vnrm)
        print(f"已导出顶点点云: {len(xyz)} 点 -> {args.output}")
        return

    tris = triangulate(faces)
    v0, v1, v2 = xyz[tris[:, 0]], xyz[tris[:, 1]], xyz[tris[:, 2]]
    total_area = float(0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1).sum())
    print(f"三角形数: {len(tris)}  表面积: {total_area:.3f}")

    if args.density is not None:
        num = max(1, int(total_area * args.density))
    else:
        num = args.num_points
    print(f"采样点数: {num}  (密度 ≈ {num/total_area:.1f} 点/㎡)")

    rng = np.random.default_rng(args.seed)
    pts, normals = sample_surface(xyz, tris, num, rng)
    write_pointcloud(args.output, pts, normals)
    print(f"已导出表面点云 -> {args.output}")


if __name__ == "__main__":
    main()
