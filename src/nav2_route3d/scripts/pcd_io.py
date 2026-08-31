from __future__ import annotations

import struct
from pathlib import Path
from typing import Iterable

import numpy as np


def read_pcd_xyz(path: str | Path) -> np.ndarray:
    """Read x/y/z from ASCII or binary PCD.

    This parser intentionally supports the common PCD variants needed by the
    graph builder. It ignores non-x/y/z fields and returns an Nx3 float32 array.
    """

    path = Path(path)
    raw = path.read_bytes()
    header_end = raw.find(b"DATA")
    if header_end < 0:
        raise ValueError(f"{path} is not a valid PCD file: missing DATA line")

    header_text_end = raw.find(b"\n", header_end)
    if header_text_end < 0:
        raise ValueError(f"{path} is not a valid PCD file: malformed DATA line")

    header = raw[: header_text_end + 1].decode("utf-8", errors="replace")
    data = raw[header_text_end + 1 :]
    metadata = _parse_header(header)
    fields = metadata.get("FIELDS", [])
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError(f"{path} PCD must contain x/y/z fields")

    data_kind = metadata.get("DATA", ["ascii"])[0].lower()
    if data_kind == "ascii":
        return _read_ascii(data.decode("utf-8", errors="replace").splitlines(), fields)
    if data_kind == "binary":
        return _read_binary(data, fields, metadata)
    if data_kind == "binary_compressed":
        return _read_binary_compressed(data, fields, metadata)
    raise ValueError(f"Unsupported PCD DATA type: {data_kind}")


def write_pcd_xyz(path: str | Path, points: np.ndarray) -> None:
    points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {points.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {points.shape[0]}\n"
        "DATA ascii\n"
    )
    lines = [f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}" for p in points]
    Path(path).write_text(header + "\n".join(lines) + "\n", encoding="utf-8")


def find_pcd_for_index(pcd_dir: str | Path, index: int) -> Path | None:
    pcd_dir = Path(pcd_dir)
    candidates = [
        pcd_dir / f"{index}.pcd",
        pcd_dir / f"{index:06d}.pcd",
        pcd_dir / f"{index:08d}.pcd",
        pcd_dir / f"pose_{index}.pcd",
        pcd_dir / f"pose_{index:06d}.pcd",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def _parse_header(header: str) -> dict[str, list[str]]:
    result: dict[str, list[str]] = {}
    for line in header.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        result[parts[0].upper()] = parts[1:]
    return result


def _read_ascii(lines: Iterable[str], fields: list[str]) -> np.ndarray:
    x_idx = fields.index("x")
    y_idx = fields.index("y")
    z_idx = fields.index("z")
    points: list[tuple[float, float, float]] = []
    for line in lines:
        parts = line.strip().split()
        if len(parts) <= max(x_idx, y_idx, z_idx):
            continue
        points.append((float(parts[x_idx]), float(parts[y_idx]), float(parts[z_idx])))
    return np.asarray(points, dtype=np.float32)


def _read_binary(data: bytes, fields: list[str], metadata: dict[str, list[str]]) -> np.ndarray:
    sizes, types, counts, points, offsets, point_step = _binary_layout(fields, metadata)
    xyz = np.empty((points, 3), dtype=np.float32)
    for i in range(points):
        base = i * point_step
        xyz[i, 0] = _unpack_field(data, base + offsets["x"], sizes[fields.index("x")], types[fields.index("x")])
        xyz[i, 1] = _unpack_field(data, base + offsets["y"], sizes[fields.index("y")], types[fields.index("y")])
        xyz[i, 2] = _unpack_field(data, base + offsets["z"], sizes[fields.index("z")], types[fields.index("z")])
    return xyz


def _read_binary_compressed(data: bytes, fields: list[str], metadata: dict[str, list[str]]) -> np.ndarray:
    if len(data) < 8:
        raise ValueError("Malformed binary_compressed PCD payload")
    compressed_size, uncompressed_size = struct.unpack_from("II", data, 0)
    compressed = data[8 : 8 + compressed_size]
    if len(compressed) != compressed_size:
        raise ValueError("Truncated binary_compressed PCD payload")
    raw = _decompress_lzf(compressed, uncompressed_size)
    sizes, types, counts, points, _offsets, point_step = _binary_layout(fields, metadata)
    if len(raw) != point_step * points:
        raise ValueError("binary_compressed PCD size does not match header")

    field_offsets: dict[str, int] = {}
    offset = 0
    for field, size, count in zip(fields, sizes, counts):
        field_offsets[field] = offset
        offset += size * count * points

    xyz = np.empty((points, 3), dtype=np.float32)
    for col, field in enumerate(("x", "y", "z")):
        idx = fields.index(field)
        stride = sizes[idx] * counts[idx]
        start = field_offsets[field]
        for i in range(points):
            xyz[i, col] = _unpack_field(raw, start + i * stride, sizes[idx], types[idx])
    return xyz


def _binary_layout(fields: list[str], metadata: dict[str, list[str]]):
    sizes = [int(v) for v in metadata["SIZE"]]
    types = metadata["TYPE"]
    counts = [int(v) for v in metadata.get("COUNT", ["1"] * len(fields))]
    points = int(metadata.get("POINTS", metadata.get("WIDTH", ["0"]))[0])
    offsets: dict[str, int] = {}
    offset = 0
    for field, size, count in zip(fields, sizes, counts):
        offsets[field] = offset
        offset += size * count
    return sizes, types, counts, points, offsets, offset


def _unpack_field(data: bytes, offset: int, size: int, type_name: str) -> float:
    return float(struct.unpack_from(_struct_format(size, type_name), data, offset)[0])


def _decompress_lzf(data: bytes, expected_size: int) -> bytes:
    out = bytearray(expected_size)
    ip = 0
    op = 0
    while ip < len(data):
        ctrl = data[ip]
        ip += 1
        if ctrl < 32:
            length = ctrl + 1
            if ip + length > len(data) or op + length > expected_size:
                raise ValueError("Invalid LZF literal run")
            out[op : op + length] = data[ip : ip + length]
            ip += length
            op += length
            continue

        length = ctrl >> 5
        ref_offset = (ctrl & 0x1F) << 8
        if length == 7:
            if ip >= len(data):
                raise ValueError("Invalid LZF back-reference length")
            length += data[ip]
            ip += 1
        if ip >= len(data):
            raise ValueError("Invalid LZF back-reference offset")
        ref_offset += data[ip]
        ip += 1
        length += 2

        ref = op - ref_offset - 1
        if ref < 0 or op + length > expected_size:
            raise ValueError("Invalid LZF back-reference")
        for _ in range(length):
            out[op] = out[ref]
            op += 1
            ref += 1

    if op != expected_size:
        raise ValueError("LZF decompressed size mismatch")
    return bytes(out)


def _struct_format(size: int, type_name: str) -> str:
    if type_name == "F" and size == 4:
        return "f"
    if type_name == "F" and size == 8:
        return "d"
    if type_name == "U" and size == 1:
        return "B"
    if type_name == "U" and size == 2:
        return "H"
    if type_name == "U" and size == 4:
        return "I"
    if type_name == "I" and size == 1:
        return "b"
    if type_name == "I" and size == 2:
        return "h"
    if type_name == "I" and size == 4:
        return "i"
    raise ValueError(f"Unsupported PCD field type {type_name} size {size}")
