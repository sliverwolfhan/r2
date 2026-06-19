"""
点云可视化 + 基础坐标系（原点坐标轴）显示

用法：
    python3 visualize_pointcloud.py                    # 显示随机示例点云
    python3 visualize_pointcloud.py --file cloud.pcd   # 加载 PCD/PLY 文件
    python3 visualize_pointcloud.py --file cloud.pcd --axis_size 0.5
"""

import argparse
import sys
import os

# 屏蔽 open3d.ml 子模块，避免触发 sklearn/scipy 的 NumPy 2.x 兼容性问题
os.environ["OPEN3D_ML_ROOT"] = ""
sys.modules["open3d.ml"] = type(sys)("open3d.ml")  # 注入空模块占位

import numpy as np
import open3d as o3d


def make_origin_frame(
    size: float = 1.0,
    translation=None,
    rotation: np.ndarray | None = None,
) -> o3d.geometry.LineSet:
    """
    创建坐标系（三条彩色轴线）
      X 轴 → 红色
      Y 轴 → 绿色
      Z 轴 → 蓝色
    translation / rotation 表示该坐标系相对 0 坐标系的位姿。
    """
    translation = np.zeros(3) if translation is None else np.asarray(translation, dtype=float)
    rotation = np.eye(3) if rotation is None else rotation

    local_points = np.array([
        [0, 0, 0],       # 原点
        [size, 0, 0],    # X
        [0, size, 0],    # Y
        [0, 0, size],    # Z
    ], dtype=float)
    points = local_points @ rotation.T + translation

    lines  = [[0, 1], [0, 2], [0, 3]]
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # R G B

    frame = o3d.geometry.LineSet()
    frame.points = o3d.utility.Vector3dVector(points)
    frame.lines  = o3d.utility.Vector2iVector(lines)
    frame.colors = o3d.utility.Vector3dVector(colors)
    return frame


def make_axis_labels(size: float = 1.0, translation=None, rotation: np.ndarray | None = None):
    """在轴末端放小球作为标记（open3d 不支持文字，用颜色区分）"""
    translation = np.zeros(3) if translation is None else np.asarray(translation, dtype=float)
    rotation = np.eye(3) if rotation is None else rotation

    spheres = []
    tips = [([size, 0, 0], [1, 0, 0]),
            ([0, size, 0], [0, 1, 0]),
            ([0, 0, size], [0, 0, 1])]
    for local_tip, color in tips:
        tip = rotation @ np.asarray(local_tip, dtype=float) + translation
        s = o3d.geometry.TriangleMesh.create_sphere(radius=size * 0.04)
        s.translate(tip)
        s.paint_uniform_color(color)
        s.compute_vertex_normals()
        spheres.append(s)
    return spheres


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """roll/pitch/yaw 分别绕 X/Y/Z 轴旋转，组合顺序为 Rz @ Ry @ Rx。"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]])
    ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0],
                   [sy, cy, 0],
                   [0, 0, 1]])

    return rz @ ry @ rx


def make_example_cloud() -> o3d.geometry.PointCloud:
    """生成一个示例点云（球面 + 平面）"""
    rng = np.random.default_rng(42)

    # 球面点云
    theta = rng.uniform(0, np.pi, 2000)
    phi   = rng.uniform(0, 2 * np.pi, 2000)
    r     = 2.0
    xs = r * np.sin(theta) * np.cos(phi)
    ys = r * np.sin(theta) * np.sin(phi)
    zs = r * np.cos(theta)

    # 地面平面点云
    gx = rng.uniform(-3, 3, 1000)
    gy = rng.uniform(-3, 3, 1000)
    gz = np.full(1000, -2.0)

    pts = np.vstack([
        np.stack([xs, ys, zs], axis=1),
        np.stack([gx, gy, gz], axis=1),
    ])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    # 按高度着色
    z_vals = pts[:, 2]
    z_norm = (z_vals - z_vals.min()) / (z_vals.max() - z_vals.min() + 1e-9)
    colors = np.stack([z_norm, 1 - z_norm, np.ones_like(z_norm) * 0.5], axis=1)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def main():
    parser = argparse.ArgumentParser(description="点云 + 坐标系可视化")
    parser.add_argument("--file",      type=str,   default=None,  help="点云文件路径 (.pcd / .ply / .xyz)")
    parser.add_argument("--axis_size", type=float, default=1.0,   help="坐标轴长度（米）")
    parser.add_argument("--point_size",type=float, default=2.0,   help="点的渲染大小")
    args = parser.parse_args()

    # 加载或生成点云
    if args.file:
        print(f"加载点云: {args.file}")
        pcd = o3d.io.read_point_cloud(args.file)
        if len(pcd.points) == 0:
            print("警告：点云为空，改用示例点云")
            pcd = make_example_cloud()
        else:
            print(f"点云加载成功，共 {len(pcd.points)} 个点")
            # 如果没有颜色，按高度着色
            if not pcd.has_colors():
                pts = np.asarray(pcd.points)
                z = pts[:, 2]
                z_norm = (z - z.min()) / (z.max() - z.min() + 1e-9)
                colors = np.stack([z_norm, 1 - z_norm, np.ones_like(z_norm) * 0.5], axis=1)
                pcd.colors = o3d.utility.Vector3dVector(colors)
    else:
        print("未指定文件，使用示例点云")
        pcd = make_example_cloud()

    # 构建 0 坐标系几何体
    frame   = make_origin_frame(size=args.axis_size)
    labels  = make_axis_labels(size=args.axis_size)

    # 构建目标坐标系：以 0 坐标系为基准，平移单位从 mm 转成 m
    target_translation = np.array([-220, -329.69, -400], dtype=float) / 1000.0
    target_rotation = euler_to_rotation_matrix(roll=0.0, pitch=0.0, yaw=3.14)
    target_frame = make_origin_frame(
        size=args.axis_size,
        translation=target_translation,
        rotation=target_rotation,
    )
    target_labels = make_axis_labels(
        size=args.axis_size,
        translation=target_translation,
        rotation=target_rotation,
    )

    # 原点小球（白色）
    origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=args.axis_size * 0.05)
    origin_sphere.paint_uniform_color([1, 1, 1])
    origin_sphere.compute_vertex_normals()

    # 目标坐标系原点小球（黄色）
    target_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=args.axis_size * 0.05)
    target_sphere.translate(target_translation)
    target_sphere.paint_uniform_color([1, 1, 0])
    target_sphere.compute_vertex_normals()

    geometries = [pcd, frame, origin_sphere, target_frame, target_sphere] + labels + target_labels

    # 渲染
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="点云 + 基础坐标系", width=1280, height=720)

    for g in geometries:
        vis.add_geometry(g)

    # 调整渲染选项
    opt = vis.get_render_option()
    opt.point_size       = args.point_size
    opt.background_color = np.array([0.1, 0.1, 0.1])  # 深灰背景
    opt.show_coordinate_frame = True  # open3d 内置坐标系（右下角小图标）

    # 设置初始视角
    ctrl = vis.get_view_control()
    ctrl.set_zoom(0.6)
    ctrl.set_front([0.5, -0.5, -0.8])
    ctrl.set_up([0, 0, 1])

    print("\n操作说明：")
    print("  鼠标左键拖动  → 旋转")
    print("  鼠标右键拖动  → 平移")
    print("  滚轮          → 缩放")
    print("  坐标轴颜色：红=X  绿=Y  蓝=Z")
    print("  按 Q 或关闭窗口退出\n")

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
