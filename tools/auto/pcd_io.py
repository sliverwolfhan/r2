"""Pure-Python PCD I/O (xyz only) — no PCL, no python-pcl dependency.

Handles ascii / binary / binary_compressed PCDs. Binary_compressed uses LZF;
the decompressor is below (small, self-contained). Designed for the field
alignment pipeline where we only need point coordinates.
"""
import struct
import numpy as np


# --------------------------------------------------------------------------- #
# LZF decompression — implements libLZF's bytecode (the format point_lio /
# pcl_io / PCL all emit). Pure Python, no extension needed.
# --------------------------------------------------------------------------- #
def _lzf_decompress(src: bytes, out_len: int) -> bytes:
    out = bytearray(out_len)
    ip, op = 0, 0
    n = len(src)
    while ip < n:
        ctrl = src[ip]
        ip += 1
        if ctrl < 32:
            # literal run of (ctrl+1) bytes
            count = ctrl + 1
            out[op:op + count] = src[ip:ip + count]
            ip += count
            op += count
        else:
            # back-reference: length in upper 3 bits, offset in lower 13
            length = ctrl >> 5
            if length == 7:
                length += src[ip]
                ip += 1
            length += 2
            ref = op - (((ctrl & 0x1f) << 8) | src[ip]) - 1
            ip += 1
            # byte-by-byte because overlapping copies are legal (RLE-style)
            for _ in range(length):
                out[op] = out[ref]
                op += 1
                ref += 1
    if op != out_len:
        raise ValueError(f"LZF decompress produced {op} bytes, expected {out_len}")
    return bytes(out)


# --------------------------------------------------------------------------- #
def _parse_header(f):
    """Read PCD ASCII header; return dict of fields + the byte offset where
    the data section starts."""
    header = {}
    while True:
        line = f.readline()
        if not line:
            raise ValueError("Unexpected EOF in PCD header")
        s = line.decode("ascii", errors="replace").strip()
        if not s or s.startswith("#"):
            continue
        key, _, val = s.partition(" ")
        key = key.upper()
        header[key] = val.strip()
        if key == "DATA":
            break
    for k in ("FIELDS", "SIZE", "TYPE", "COUNT", "POINTS", "DATA"):
        if k not in header:
            raise ValueError(f"PCD header missing {k}")
    header["FIELDS"] = header["FIELDS"].split()
    header["SIZE"] = [int(v) for v in header["SIZE"].split()]
    header["TYPE"] = header["TYPE"].split()
    header["COUNT"] = [int(v) for v in header["COUNT"].split()]
    header["POINTS"] = int(header["POINTS"])
    return header, f.tell()


def _np_dtype(t, size):
    if t == "F":
        return {4: np.float32, 8: np.float64}[size]
    if t == "I":
        return {1: np.int8, 2: np.int16, 4: np.int32, 8: np.int64}[size]
    if t == "U":
        return {1: np.uint8, 2: np.uint16, 4: np.uint32, 8: np.uint64}[size]
    raise ValueError(f"Unknown PCD type {t!r}")


def read_pcd_xyz(path: str) -> np.ndarray:
    """Return Nx3 float64 array of xyz points from any PCD (ascii / binary /
    binary_compressed). Other fields are skipped."""
    with open(path, "rb") as f:
        hdr, data_off = _parse_header(f)

        try:
            ix = hdr["FIELDS"].index("x")
            iy = hdr["FIELDS"].index("y")
            iz = hdr["FIELDS"].index("z")
        except ValueError:
            raise ValueError("PCD has no x/y/z fields")
        n = hdr["POINTS"]
        data_fmt = hdr["DATA"].lower()

        if data_fmt == "ascii":
            xyz = np.empty((n, 3), float)
            for i in range(n):
                parts = f.readline().split()
                xyz[i, 0] = float(parts[ix])
                xyz[i, 1] = float(parts[iy])
                xyz[i, 2] = float(parts[iz])
            return xyz

        # binary forms — build a structured dtype matching the row layout
        names, formats, offsets = [], [], []
        off = 0
        for idx, (name, t, sz, cnt) in enumerate(
            zip(hdr["FIELDS"], hdr["TYPE"], hdr["SIZE"], hdr["COUNT"])
        ):
            unique = f"{name}_{idx}"   # PCD can repeat field names; numpy can't
            names.append(unique)
            base = _np_dtype(t, sz)
            formats.append((base, (cnt,)) if cnt > 1 else base)
            offsets.append(off)
            off += sz * cnt
        row_dtype = np.dtype({"names": names, "formats": formats,
                              "offsets": offsets, "itemsize": off})

        if data_fmt == "binary":
            raw = f.read(off * n)
            arr = np.frombuffer(raw, dtype=row_dtype, count=n)
            xyz = np.column_stack([arr[names[ix]], arr[names[iy]], arr[names[iz]]])
            return xyz.astype(float)

        if data_fmt == "binary_compressed":
            comp_sz, uncomp_sz = struct.unpack("<II", f.read(8))
            comp = f.read(comp_sz)
            blob = _lzf_decompress(comp, uncomp_sz)
            # binary_compressed is SoA: all x's, then all y's, then all z's...
            cursor = 0
            cols = {}
            for name, t, sz, cnt in zip(hdr["FIELDS"], hdr["TYPE"],
                                        hdr["SIZE"], hdr["COUNT"]):
                base = _np_dtype(t, sz)
                col_bytes = sz * cnt * n
                if name in ("x", "y", "z") and cnt == 1:
                    cols[name] = np.frombuffer(blob, dtype=base, count=n,
                                               offset=cursor).astype(float)
                cursor += col_bytes
            return np.column_stack([cols["x"], cols["y"], cols["z"]])

    raise ValueError(f"Unsupported DATA format: {hdr['DATA']!r}")


# --------------------------------------------------------------------------- #
def write_pcd_xyz(path: str, xyz: np.ndarray):
    """Write a binary little-endian XYZ float32 PCD. Sufficient for downstream
    GICP / visualisation tools."""
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("xyz must be Nx3")
    n = len(xyz)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    ).encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(xyz.tobytes(order="C"))
