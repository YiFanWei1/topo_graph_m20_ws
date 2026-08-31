#!/usr/bin/env python3
"""
graph_editor.py - nav2_route3d 3D graph editor
─────────────────────────────────────────────
Visual editor for nav2_route3d GeoJSON/JSON graph files and PCD frame maps.

Features
  - Load/save graph_3d.json (GeoJSON FeatureCollection).
  - Optional PCD map overlay (background only — not selectable, low contrast).
  - Add node            : LMB click on empty space (when in Add Node mode)
  - Delete node         : select node + Delete key, or in Delete Node mode click
  - Create edge         : LMB drag from one node to another (any mode)
  - Delete edge         : RMB drag across an edge (highlights then removes)
  - Toggle node IDs     : View → Show Node IDs
  - Edit metadata       : double-click node/edge → JSON dialog

Ubuntu 24.04, ROS 2 Jazzy, Python ≥ 3.10.

Required pip packages:
    pip install PyQt6 pyvista pyvistaqt numpy

System packages (Wayland desktop, Ubuntu 24.04):
    sudo apt install libxcb-cursor0
"""
from __future__ import annotations
import argparse
import ctypes.util
import json
import os
import sys
import uuid
import struct


def _libxcb_cursor_available() -> bool:
    if ctypes.util.find_library("xcb-cursor") is not None:
        return True
    for path in (
        "/usr/lib/x86_64-linux-gnu/libxcb-cursor.so.0",
        "/usr/lib/aarch64-linux-gnu/libxcb-cursor.so.0",
    ):
        if os.path.isfile(path):
            return True
    return False


def _configure_qt_platform() -> None:
    """PyVistaQt/VTK does not work on native Wayland; use XWayland (xcb) instead."""
    if os.environ.get("QT_QPA_PLATFORM"):
        return
    if not os.environ.get("DISPLAY"):
        return
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    if not _libxcb_cursor_available():
        print(
            "graph_editor requires the X11 (xcb) Qt plugin on Wayland desktops.\n"
            "Install the missing system library, then rerun:\n\n"
            "  sudo apt install libxcb-cursor0\n\n"
            "PyVistaQt/VTK is not compatible with native Wayland (BadWindow).",
            file=sys.stderr,
        )
        sys.exit(2)


from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtCore import Qt, QTimer, QEvent
from PyQt6.QtGui import QAction, QKeySequence
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QMessageBox, QTextEdit,
    QDialog, QDialogButtonBox, QVBoxLayout, QLabel, QStatusBar,
    QToolBar, QButtonGroup, QPushButton, QHBoxLayout, QWidget,
)

import pyvista as pv
import vtk
from pyvistaqt import QtInteractor


def lzf_decompress(data: bytes, expected_size: int) -> bytes:
    """Decompress PCL PCD binary_compressed payloads, which use LZF."""
    out = bytearray(expected_size)
    ip = 0
    op = 0
    data_len = len(data)
    while ip < data_len:
        ctrl = data[ip]
        ip += 1
        if ctrl < 32:
            length = ctrl + 1
            if op + length > expected_size or ip + length > data_len:
                raise ValueError("Invalid LZF literal run in compressed PCD")
            out[op:op + length] = data[ip:ip + length]
            op += length
            ip += length
        else:
            length = ctrl >> 5
            ref_offset = (ctrl & 0x1F) << 8
            if ip >= data_len:
                raise ValueError("Invalid LZF back-reference in compressed PCD")
            if length == 7:
                length += data[ip]
                ip += 1
                if ip >= data_len:
                    raise ValueError("Invalid LZF back-reference length in compressed PCD")
            ref_offset += data[ip]
            ip += 1
            ref = op - ref_offset - 1
            length += 2
            if ref < 0 or op + length > expected_size:
                raise ValueError("Invalid LZF back-reference range in compressed PCD")
            for _ in range(length):
                out[op] = out[ref]
                op += 1
                ref += 1
    if op != expected_size:
        raise ValueError("Compressed PCD payload size mismatch")
    return bytes(out)


def read_pcd_xyz_array(path: str) -> np.ndarray:
    """Read enough of a PCD file for visualization and return xyz points."""
    with open(path, "rb") as f:
        header_lines: list[str] = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD header is missing DATA line")
            decoded = line.decode("utf-8", errors="replace").strip()
            header_lines.append(decoded)
            if decoded.startswith("DATA "):
                break
        payload = f.read()

    header: dict[str, list[str]] = {}
    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        header[parts[0]] = parts[1:]

    fields = header.get("FIELDS", [])
    sizes = [int(v) for v in header.get("SIZE", [])]
    counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
    types = header.get("TYPE", [])
    points = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
    data_kind = header.get("DATA", [""])[0]

    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PCD must contain x, y and z fields")
    if any(t != "F" or s != 4 for t, s in zip(types, sizes)):
        raise ValueError("Only 32-bit float PCD fields are supported by the fallback reader")

    offsets: dict[str, int] = {}
    offset = 0
    for field, size, count in zip(fields, sizes, counts):
        offsets[field] = offset
        offset += size * count
    point_step = offset

    expected = points * point_step
    xyz = np.empty((points, 3), dtype=np.float32)

    if data_kind == "binary_compressed":
        if len(payload) < 8:
            raise ValueError("Compressed PCD payload is truncated")
        compressed_size, uncompressed_size = struct.unpack("<II", payload[:8])
        compressed = payload[8:8 + compressed_size]
        raw = lzf_decompress(compressed, uncompressed_size)
        if len(raw) < expected:
            raise ValueError("PCD payload is shorter than expected")

        # PCL stores binary_compressed PCD data field-by-field, not point-by-point.
        field_bases: dict[str, int] = {}
        base = 0
        for field, size, count in zip(fields, sizes, counts):
            field_bases[field] = base
            base += size * count * points
        xyz[:, 0] = np.frombuffer(raw, dtype="<f4", count=points, offset=field_bases["x"])
        xyz[:, 1] = np.frombuffer(raw, dtype="<f4", count=points, offset=field_bases["y"])
        xyz[:, 2] = np.frombuffer(raw, dtype="<f4", count=points, offset=field_bases["z"])
    elif data_kind == "binary":
        raw = payload
        if len(raw) < expected:
            raise ValueError("PCD payload is shorter than expected")
        xyz[:, 0] = np.ndarray((points,), dtype="<f4", buffer=raw, offset=offsets["x"], strides=(point_step,))
        xyz[:, 1] = np.ndarray((points,), dtype="<f4", buffer=raw, offset=offsets["y"], strides=(point_step,))
        xyz[:, 2] = np.ndarray((points,), dtype="<f4", buffer=raw, offset=offsets["z"], strides=(point_step,))
    else:
        cloud = pv.read(path)
        return np.asarray(cloud.points, dtype=np.float32)

    return xyz


def read_pcd_xyz(path: str) -> pv.PolyData:
    return pv.PolyData(read_pcd_xyz_array(path))


def load_pose_rows(path: Path) -> list[tuple[float, np.ndarray, np.ndarray]]:
    text = path.read_text().strip()
    poses: list[tuple[float, np.ndarray, np.ndarray]] = []
    if not text:
        return poses

    if text[0] in "[{":
        doc = json.loads(text)
        rows = doc["poses"] if isinstance(doc, dict) and "poses" in doc else doc
        for row in rows:
            poses.append((
                float(row[0]),
                np.array([row[1], row[2], row[3]], dtype=np.float32),
                np.array([row[4], row[5], row[6], row[7]], dtype=np.float32),
            ))
    else:
        for line in text.splitlines():
            if not line.strip():
                continue
            row = [float(v) for v in line.split()]
            if len(row) < 8:
                raise ValueError(f"Pose row must have 8 columns: {line}")
            poses.append((
                row[0],
                np.array(row[1:4], dtype=np.float32),
                np.array(row[4:8], dtype=np.float32),
            ))
    return poses


def quaternion_matrix_xyzw(q: np.ndarray) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    n = x * x + y * y + z * z + w * w
    if n <= 0.0:
        return np.eye(3, dtype=np.float32)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ], dtype=np.float32)


def downsample_points(points: np.ndarray, rate: float, rng: np.random.Generator) -> np.ndarray:
    if rate >= 1.0 or points.size == 0:
        return points
    keep = max(1, int(points.shape[0] * max(rate, 0.0)))
    if keep >= points.shape[0]:
        return points
    idx = rng.choice(points.shape[0], size=keep, replace=False)
    return points[idx]


def sorted_pcd_files(directory: Path) -> list[Path]:
    def key(path: Path):
        try:
            return (0, int(path.stem))
        except ValueError:
            return (1, path.name)
    return sorted(directory.glob("*.pcd"), key=key)


def infer_pose_file(pcd_dir: Path) -> Path:
    for candidate in (pcd_dir.parent / "pose.json", pcd_dir.parent / "poses.json"):
        if candidate.exists():
            return candidate
    raise ValueError(f"No pose.json or poses.json found next to {pcd_dir}")


def load_pcd_overlay(path: str, downsample_rate: float, max_points: int) -> pv.PolyData:
    source = Path(path).expanduser()
    rng = np.random.default_rng(7)

    if source.is_file():
        points = downsample_points(read_pcd_xyz_array(str(source)), downsample_rate, rng)
        if max_points > 0 and points.shape[0] > max_points:
            points = downsample_points(points, max_points / points.shape[0], rng)
        return pv.PolyData(points)

    if not source.is_dir():
        raise ValueError(f"PCD path does not exist: {source}")

    pose_rows = load_pose_rows(infer_pose_file(source))
    pcd_files = sorted_pcd_files(source)
    if not pcd_files:
        raise ValueError(f"No .pcd files found in {source}")
    count = min(len(pose_rows), len(pcd_files))
    if count == 0:
        raise ValueError("No matching pose/PCD frames found")

    chunks: list[np.ndarray] = []
    for i in range(count):
        _, pos, quat = pose_rows[i]
        points = downsample_points(read_pcd_xyz_array(str(pcd_files[i])), downsample_rate, rng)
        rot = quaternion_matrix_xyzw(quat)
        chunks.append(points @ rot.T + pos)

    merged = np.vstack(chunks).astype(np.float32, copy=False)
    if max_points > 0 and merged.shape[0] > max_points:
        merged = downsample_points(merged, max_points / merged.shape[0], rng)
    return pv.PolyData(merged)


# ─────────────────────────────────────────────────────────────────────────────
#  Data model
# ─────────────────────────────────────────────────────────────────────────────
class GraphModel:
    """In-memory representation of nav2_route3d GeoJSON/JSON graphs."""

    def __init__(self):
        self.nodes: dict[int, dict] = {}   # id -> {"pos":[x,y,z], "metadata":{...}}
        self.edges: dict[int, dict] = {}   # id -> {"from":int, "to":int, "metadata":{...}}
        self.frame_id = "map"
        self.graph_metadata: dict = {}
        self._next_node_id = 0
        self._next_edge_id = 1

    # ── load / save ───────────────────────────────────────────────────────
    def load(self, path: str) -> None:
        with open(path) as f:
            doc = json.load(f)
        self.nodes.clear()
        self.edges.clear()
        self._next_node_id = 0
        self._next_edge_id = 1
        if doc.get("type") == "FeatureCollection":
            self._load_geojson(doc)
        else:
            self._load_nav2_json(doc)

    def _load_geojson(self, doc: dict) -> None:
        props_root = doc.get("properties", {})
        self.frame_id = props_root.get("frame_id", doc.get("frame_id", "map"))
        self.graph_metadata = props_root.get("metadata", doc.get("metadata", {}))
        if not isinstance(self.graph_metadata, dict):
            self.graph_metadata = {}
        for feat in doc.get("features", []):
            props = feat.get("properties", {})
            kind = props.get("kind")
            if kind == "node":
                nid = int(props.get("id", feat.get("id")))
                coords = feat.get("geometry", {}).get("coordinates", [0.0, 0.0, 0.0])
                metadata = props.get("metadata", {})
                self.nodes[nid] = {
                    "pos": [float(coords[0]), float(coords[1]), float(coords[2] if len(coords) > 2 else 0.0)],
                    "metadata": metadata if isinstance(metadata, dict) else {},
                    "timestamp": float(props.get("timestamp", 0.0)),
                    "orientation": list(props.get("orientation", [0.0, 0.0, 0.0, 1.0])),
                    "real_neighbors": list(props.get("real_neighbors", [])),
                    "fake_neighbors": list(props.get("fake_neighbors", [])),
                }
                self._next_node_id = max(self._next_node_id, nid + 1)
            elif kind == "edge":
                eid = int(props.get("id", feat.get("id")))
                start = props.get("start_id", props.get("from"))
                end = props.get("end_id", props.get("to"))
                if start is None or end is None:
                    continue
                metadata = props.get("metadata", {})
                self.edges[eid] = {
                    "from": int(start),
                    "to": int(end),
                    "metadata": metadata if isinstance(metadata, dict) else {},
                    "cost": float(props.get("cost", 0.0)),
                    "bidirectional": bool(props.get("bidirectional", True)),
                    "operations": list(props.get("operations", [])),
                }
                self._next_edge_id = max(self._next_edge_id, eid + 1)

    def _load_nav2_json(self, doc: dict) -> None:
        self.frame_id = doc.get("frame_id", "map")
        metadata = doc.get("metadata", {})
        self.graph_metadata = metadata if isinstance(metadata, dict) else {}
        for item in doc.get("nodes", []):
            nid = int(item["id"])
            row = item.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            metadata = item.get("metadata", {})
            self.nodes[nid] = {
                "pos": [float(row[1]), float(row[2]), float(row[3])],
                "metadata": metadata if isinstance(metadata, dict) else {},
                "timestamp": float(row[0]),
                "orientation": [float(row[4]), float(row[5]), float(row[6]), float(row[7])],
                "real_neighbors": list(item.get("real_neighbors", [])),
                "fake_neighbors": list(item.get("fake_neighbors", [])),
            }
            self._next_node_id = max(self._next_node_id, nid + 1)
        for item in doc.get("edges", []):
            eid = int(item["id"])
            metadata = item.get("metadata", {})
            self.edges[eid] = {
                "from": int(item["start_id"]),
                "to": int(item["end_id"]),
                "metadata": metadata if isinstance(metadata, dict) else {},
                "cost": float(item.get("cost", 0.0)),
                "bidirectional": bool(item.get("bidirectional", True)),
                "operations": list(item.get("operations", [])),
            }
            self._next_edge_id = max(self._next_edge_id, eid + 1)

    def save(self, path: str) -> None:
        features = []
        for nid, n in sorted(self.nodes.items()):
            orientation = n.get("orientation", [0.0, 0.0, 0.0, 1.0])
            features.append({
                "id": nid,
                "type": "Feature",
                "geometry": {"type": "Point",
                              "coordinates": [n["pos"][0], n["pos"][1], n["pos"][2]]},
                "properties": {
                    "kind": "node",
                    "id": nid,
                    "timestamp": n.get("timestamp", 0.0),
                    "orientation": orientation,
                    "metadata": n.get("metadata", {}),
                    "real_neighbors": sorted(set(int(v) for v in n.get("real_neighbors", []))),
                    "fake_neighbors": sorted(set(int(v) for v in n.get("fake_neighbors", []))),
                },
            })
        for eid, e in sorted(self.edges.items()):
            a = self.nodes[e["from"]]["pos"]
            b = self.nodes[e["to"]]["pos"]
            features.append({
                "id": eid,
                "type": "Feature",
                "geometry": {"type": "LineString",
                              "coordinates": [
                                [a[0], a[1], a[2]],
                                [b[0], b[1], b[2]],
                              ]},
                "properties": {
                    "kind": "edge",
                    "id": eid,
                    "from": e["from"],
                    "to": e["to"],
                    "start_id": e["from"],
                    "end_id": e["to"],
                    "cost": e.get("cost", 0.0),
                    "bidirectional": e.get("bidirectional", True),
                    "metadata": e.get("metadata", {}),
                    "operations": e.get("operations", []),
                },
            })
        doc = {
            "type": "FeatureCollection",
            "name": "nav2_route3d_graph",
            "properties": {"frame_id": self.frame_id, "metadata": self.graph_metadata},
            "features": features,
        }
        with open(path, "w") as f:
            json.dump(doc, f, indent=2)

    # ── mutation ──────────────────────────────────────────────────────────
    def add_node(self, pos, metadata=None) -> int:
        nid = self._next_node_id
        self._next_node_id += 1
        self.nodes[nid] = {"pos": list(pos), "metadata": metadata or {}}
        return nid

    def remove_node(self, nid: int) -> None:
        # Remove all incident edges first
        rm = [eid for eid, e in self.edges.items()
              if e["from"] == nid or e["to"] == nid]
        for eid in rm:
            del self.edges[eid]
        self.nodes.pop(nid, None)

    def add_edge(self, a: int, b: int, metadata=None) -> Optional[int]:
        if a == b or a not in self.nodes or b not in self.nodes:
            return None
        # avoid duplicate parallel edges
        for e in self.edges.values():
            if e["from"] == a and e["to"] == b:
                return None
        eid = self._next_edge_id
        self._next_edge_id += 1
        self.edges[eid] = {"from": a, "to": b,
                            "metadata": metadata or {"type": "walk"}}
        return eid

    def remove_edge(self, eid: int) -> None:
        self.edges.pop(eid, None)


# ─────────────────────────────────────────────────────────────────────────────
#  Metadata edit dialog
# ─────────────────────────────────────────────────────────────────────────────
class MetadataDialog(QDialog):
    def __init__(self, title: str, data: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(450, 350)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Edit metadata as JSON object:"))
        self.text = QTextEdit()
        self.text.setPlainText(json.dumps(data, indent=2, ensure_ascii=False))
        layout.addWidget(self.text)
        bb = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        bb.accepted.connect(self.on_ok)
        bb.rejected.connect(self.reject)
        layout.addWidget(bb)
        self._result: Optional[dict] = None

    def on_ok(self):
        try:
            self._result = json.loads(self.text.toPlainText())
            if not isinstance(self._result, dict):
                raise ValueError("metadata must be a JSON object")
            self.accept()
        except (json.JSONDecodeError, ValueError) as e:
            QMessageBox.warning(self, "Invalid JSON", str(e))

    def result_data(self) -> Optional[dict]:
        return self._result


# ─────────────────────────────────────────────────────────────────────────────
#  Editor window
# ─────────────────────────────────────────────────────────────────────────────
class GraphEditor(QMainWindow):

    # Editor modes
    MODE_VIEW          = "view"
    MODE_ADD_NODE      = "add_node"
    MODE_ADD_EDGE      = "add_edge"
    MODE_DELETE_NODE   = "delete_node"
    MODE_DELETE_EDGE   = "delete_edge"

    def __init__(self, pcd_downsample_rate: float = 0.02, pcd_max_points: int = 1000000):
        super().__init__()
        self.pcd_downsample_rate = pcd_downsample_rate
        self.pcd_max_points = pcd_max_points
        self.setWindowTitle("nav2_route3d 3D graph editor")
        self.resize(1280, 800)

        self.model    = GraphModel()
        self.mode     = self.MODE_VIEW
        self.graph_path: Optional[str] = None
        self.pcd_path: Optional[str]   = None
        self.show_ids = False

        # Pick state for drag-to-create-edge
        self._edge_drag_source: Optional[int] = None
        self._graph_mouse_press_handled = False

        # Actor handles (so we can remove and replace efficiently)
        self._node_actor = None
        self._edge_actor = None
        self._label_actor = None
        self._pcd_actor   = None
        self._highlight_actor = None

        # Picking maps: actor cell index → object id
        self._node_picker_cell_to_id: dict[int, int] = {}
        self._edge_picker_cell_to_id: dict[int, int] = {}

        central = QWidget()
        layout = QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        self.plotter = QtInteractor(central)
        layout.addWidget(self.plotter.interactor)
        self.setCentralWidget(central)

        # ── UI layout ─────────────────────────────────────────────────────
        self._build_menus()
        self._build_toolbar()

        self.setStatusBar(QStatusBar())
        self._set_status("ready")

        # Interaction is handled with explicit screen-space picking below. This is
        # more reliable than actor picking for dense PyVista/VTK scenes.
        self.plotter.interactor.installEventFilter(self)

        # Style: dark bg keeps graph high-contrast
        self.plotter.set_background("#1a1a1a")
        self.plotter.add_axes()

    # ── Menus ─────────────────────────────────────────────────────────────
    def _build_menus(self):
        mb = self.menuBar()
        m_file = mb.addMenu("&File")
        a_open = QAction("Open Graph...", self); a_open.setShortcut("Ctrl+O")
        a_open.triggered.connect(self.open_graph)
        m_file.addAction(a_open)

        a_save = QAction("Save Graph", self); a_save.setShortcut("Ctrl+S")
        a_save.triggered.connect(self.save_graph)
        m_file.addAction(a_save)

        a_saveas = QAction("Save Graph As...", self); a_saveas.setShortcut("Ctrl+Shift+S")
        a_saveas.triggered.connect(self.save_graph_as)
        m_file.addAction(a_saveas)

        m_file.addSeparator()
        a_pcd = QAction("Load PCD Frame...", self)
        a_pcd.triggered.connect(self.open_pcd)
        m_file.addAction(a_pcd)
        a_pcd_dir = QAction("Load PCD Directory...", self)
        a_pcd_dir.triggered.connect(self.open_pcd_dir)
        m_file.addAction(a_pcd_dir)
        a_pcd_clear = QAction("Clear PCD Map", self)
        a_pcd_clear.triggered.connect(self.clear_pcd)
        m_file.addAction(a_pcd_clear)

        m_view = mb.addMenu("&View")
        self.a_show_ids = QAction("Show Node IDs", self, checkable=True)
        self.a_show_ids.toggled.connect(self.toggle_ids)
        m_view.addAction(self.a_show_ids)

        a_reset = QAction("Reset Camera", self); a_reset.setShortcut("R")
        a_reset.triggered.connect(self.reset_camera_view)
        m_view.addAction(a_reset)

    # ── Toolbar (mode switching) ──────────────────────────────────────────
    def _build_toolbar(self):
        tb = QToolBar()
        self.addToolBar(tb)
        self.btn_group = QButtonGroup(self); self.btn_group.setExclusive(True)
        for label, mode in [
            ("View",        self.MODE_VIEW),
            ("Add Node",    self.MODE_ADD_NODE),
            ("Add Edge",    self.MODE_ADD_EDGE),
            ("Delete Node", self.MODE_DELETE_NODE),
            ("Delete Edge", self.MODE_DELETE_EDGE),
        ]:
            b = QPushButton(label); b.setCheckable(True)
            if mode == self.MODE_VIEW: b.setChecked(True)
            b.clicked.connect(lambda _, m=mode: self.set_mode(m))
            tb.addWidget(b)
            self.btn_group.addButton(b)

    # ── Status helpers ────────────────────────────────────────────────────
    def _set_status(self, msg: str):
        self.statusBar().showMessage(msg)

    def _refresh_camera_clipping(self):
        try:
            self.plotter.reset_camera_clipping_range()
            self.plotter.camera.SetClippingRange(0.001, 1.0e9)
        except Exception:
            pass

    def reset_camera_view(self):
        self.plotter.reset_camera()
        self._refresh_camera_clipping()
        self.plotter.update()

    def set_top_down_view(self):
        if not self.model.nodes:
            return self.reset_camera_view()
        pts = np.array([node["pos"] for node in self.model.nodes.values()], dtype=np.float64)
        center = pts.mean(axis=0)
        span = np.ptp(pts, axis=0)
        distance = max(float(span[0]), float(span[1]), 1.0) * 1.6
        self.plotter.camera_position = [
            (float(center[0]), float(center[1]), float(center[2] + distance)),
            (float(center[0]), float(center[1]), float(center[2])),
            (0.0, 1.0, 0.0),
        ]
        self._refresh_camera_clipping()
        self.plotter.update()
        self._set_status("top-down view")

    def set_mode(self, mode: str):
        self.mode = mode
        self._edge_drag_source = None
        self._set_status(f"mode: {mode}")

    # ── File ops ──────────────────────────────────────────────────────────
    def open_graph(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open graph_3d.json",
            filter="GeoJSON (*.json *.geojson)")
        if not path: return
        try:
            self.model.load(path)
            self.graph_path = path
            self.redraw_graph()
            self._set_status(f"loaded {len(self.model.nodes)} nodes / "
                              f"{len(self.model.edges)} edges from {path}")
        except Exception as e:
            QMessageBox.critical(self, "Load failed", str(e))

    def save_graph(self):
        if not self.graph_path: return self.save_graph_as()
        try:
            self.model.save(self.graph_path)
            self._set_status(f"saved → {self.graph_path}")
        except Exception as e:
            QMessageBox.critical(self, "Save failed", str(e))

    def save_graph_as(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save graph",
            filter="GeoJSON (*.json)")
        if not path: return
        if not path.endswith(".json"): path += ".json"
        self.graph_path = path
        self.save_graph()

    def open_pcd(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open PCD map", filter="PCD (*.pcd)")
        if not path: return
        try:
            self.pcd_path = path
            self.redraw_pcd()
            self._set_status(f"loaded PCD: {path}")
        except Exception as e:
            QMessageBox.critical(self, "PCD load failed", str(e))

    def open_pcd_dir(self):
        path = QFileDialog.getExistingDirectory(self, "Open PCD frame directory")
        if not path: return
        try:
            self.pcd_path = path
            self.redraw_pcd()
        except Exception as e:
            QMessageBox.critical(self, "PCD load failed", str(e))

    def clear_pcd(self):
        if self._pcd_actor is not None:
            self.plotter.remove_actor(self._pcd_actor)
            self._pcd_actor = None
        self.pcd_path = None
        self._set_status("PCD cleared")

    # ── Rendering ─────────────────────────────────────────────────────────
    def redraw_pcd(self):
        if self._pcd_actor is not None:
            self.plotter.remove_actor(self._pcd_actor)
            self._pcd_actor = None
        if not self.pcd_path: return
        try:
            cloud = load_pcd_overlay(self.pcd_path, self.pcd_downsample_rate, self.pcd_max_points)
        except Exception as e:
            QMessageBox.critical(self, "PCD read failed", str(e))
            return
        # Make the PCD intentionally subdued: small points, low-contrast gray,
        # not pickable.  Graph stays the visual focus.
        self._pcd_actor = self.plotter.add_mesh(
            cloud,
            color="#555555",
            point_size=1.2,
            opacity=0.45,
            render_points_as_spheres=False,
            pickable=False,                # essential — PCD is never selectable
            name="pcd_map",
        )
        self._set_status(f"loaded PCD overlay: {cloud.n_points} points")
        self._refresh_camera_clipping()
        self.plotter.update()

    def redraw_graph(self):
        # Remove old actors
        for a in (self._node_actor, self._edge_actor, self._label_actor):
            if a is not None:
                self.plotter.remove_actor(a)
        self._node_actor = self._edge_actor = self._label_actor = None
        self._node_picker_cell_to_id.clear()
        self._edge_picker_cell_to_id.clear()

        # ── Nodes ────────────────────────────────────────────────────────
        if self.model.nodes:
            ids_sorted = sorted(self.model.nodes.keys())
            pts = np.array([self.model.nodes[i]["pos"] for i in ids_sorted],
                           dtype=np.float64)
            cloud = pv.PolyData(pts)
            # Add an array so we can scalar-color nodes selectively later
            cloud["node_id"] = np.array(ids_sorted, dtype=np.int64)
            self._node_actor = self.plotter.add_mesh(
                cloud,
                color="#5BD0FF",
                point_size=14,
                render_points_as_spheres=True,
                pickable=True,
                name="graph_nodes",
            )
            # Picker maps point index → node id
            for i, nid in enumerate(ids_sorted):
                self._node_picker_cell_to_id[i] = nid

            # Labels
            if self.show_ids:
                self._label_actor = self.plotter.add_point_labels(
                    pts,
                    [str(nid) for nid in ids_sorted],
                    font_size=12,
                    text_color="white",
                    point_color=None,
                    point_size=0,
                    shape_opacity=0.3,
                    always_visible=True,
                    pickable=False,
                    name="graph_labels",
                )

        # ── Edges ────────────────────────────────────────────────────────
        if self.model.edges:
            eids_sorted = sorted(self.model.edges.keys())
            lines_pts = []
            lines_conn = []
            for k, eid in enumerate(eids_sorted):
                e = self.model.edges[eid]
                a = self.model.nodes[e["from"]]["pos"]
                b = self.model.nodes[e["to"]]["pos"]
                lines_pts.append(a)
                lines_pts.append(b)
                lines_conn.extend([2, 2*k, 2*k + 1])
            mesh = pv.PolyData(np.array(lines_pts, dtype=np.float64),
                                lines=np.array(lines_conn, dtype=np.int64))
            mesh["edge_id"] = np.array(eids_sorted, dtype=np.int64)
            self._edge_actor = self.plotter.add_mesh(
                mesh,
                color="#FFA94D",
                line_width=3,
                pickable=True,
                name="graph_edges",
            )
            for i, eid in enumerate(eids_sorted):
                self._edge_picker_cell_to_id[i] = eid

        # Trigger render
        self._refresh_camera_clipping()
        self.plotter.update()

    def toggle_ids(self, on: bool):
        self.show_ids = on
        self.redraw_graph()

    # ── Picking & interaction ─────────────────────────────────────────────
    def _on_pick(self, mesh):
        """pyvistaqt picker — called with picked mesh actor when user clicks."""
        if mesh is None: return

        # Inspect the actor name to know what we picked
        # pyvista exposes picked_cell / picked_point depending on geometry
        try:
            picked_cell = self.plotter.picked_cell
            picked_point = self.plotter.picked_point
        except Exception:
            picked_cell = picked_point = None

        # We routed picker via add_mesh_picking and mesh_picking returns the
        # active mesh ↔ id; we use mesh name to dispatch.
        name = getattr(mesh, "name", "") or ""
        if name == "graph_nodes":
            # picked_point is index into the point list
            idx = getattr(self.plotter, "picked_point", None)
            if idx is None: return
            try:
                nid = list(sorted(self.model.nodes.keys()))[int(idx)]
            except Exception:
                return
            self._on_node_picked(nid)
        elif name == "graph_edges":
            idx = getattr(self.plotter, "picked_cell", None)
            if idx is None: return
            try:
                eid = list(sorted(self.model.edges.keys()))[int(idx)]
            except Exception:
                return
            self._on_edge_picked(eid)
        # If user clicked empty space in Add Node mode, we'll handle that
        # via a left-button click handler bound separately below in keyPressEvent.

    def _release_camera_interaction(self):
        self._graph_mouse_press_handled = False
        try:
            self.plotter.interactor.releaseMouse()
        except Exception:
            pass
        try:
            interactor = self.plotter.iren.interactor
            interactor.LeftButtonReleaseEvent()
            interactor.RightButtonReleaseEvent()
            interactor.MiddleButtonReleaseEvent()
            interactor.MouseMoveEvent()
        except Exception:
            pass

    def edit_node_metadata(self, nid: int):
        node = self.model.nodes.get(nid)
        if node is None:
            return
        dlg = MetadataDialog(f"Node {nid} metadata", node.get("metadata", {}), self)
        try:
            if dlg.exec() == QDialog.DialogCode.Accepted:
                data = dlg.result_data()
                if data is not None:
                    node["metadata"] = data
                    self._set_status(f"node {nid} metadata updated (valid JSON)")
        finally:
            self._release_camera_interaction()

    def edit_edge_metadata(self, eid: int):
        edge = self.model.edges.get(eid)
        if edge is None:
            return
        dlg = MetadataDialog(f"Edge {eid} metadata", edge.get("metadata", {}), self)
        try:
            if dlg.exec() == QDialog.DialogCode.Accepted:
                data = dlg.result_data()
                if data is not None:
                    edge["metadata"] = data
                    self._set_status(f"edge {eid} metadata updated (valid JSON)")
        finally:
            self._release_camera_interaction()

    def _on_node_picked(self, nid: int):
        if self.mode == self.MODE_DELETE_NODE:
            self.model.remove_node(nid)
            self.redraw_graph()
            self._set_status(f"deleted node {nid}")
        elif self.mode == self.MODE_VIEW:
            self.edit_node_metadata(nid)
        elif self.mode == self.MODE_ADD_EDGE:
            # Start edge-drag from this node; if a second node is picked
            # while drag source set, create edge.
            if self._edge_drag_source is None:
                self._edge_drag_source = nid
                self._set_status(f"edge source = {nid}; pick target node")
            else:
                src = self._edge_drag_source
                self._edge_drag_source = None
                if src == nid:
                    self._set_status("edge canceled (same node)")
                    return
                eid = self.model.add_edge(src, nid)
                if eid is None:
                    self._set_status("edge not added (duplicate or invalid)")
                else:
                    self.redraw_graph()
                    self._set_status(f"added edge {eid}: {src} → {nid}")

    def _on_edge_picked(self, eid: int):
        if self.mode == self.MODE_DELETE_EDGE:
            self.model.remove_edge(eid)
            self.redraw_graph()
            self._set_status(f"deleted edge {eid}")
        elif self.mode == self.MODE_VIEW:
            self.edit_edge_metadata(eid)

    def _vtk_actor(self, actor):
        if actor is None:
            return None
        return getattr(actor, "actor", actor)

    def _same_actor(self, a, b) -> bool:
        a = self._vtk_actor(a)
        b = self._vtk_actor(b)
        if a is None or b is None:
            return False
        try:
            return a == b or a.GetAddressAsString("") == b.GetAddressAsString("")
        except Exception:
            return a == b

    def _event_pick_xy(self, ev) -> tuple[int, int]:
        pos = ev.position() if hasattr(ev, "position") else ev.pos()
        x = int(pos.x())
        y = int(self.plotter.interactor.height() - pos.y())
        return x, y

    def _vtk_renderer(self):
        return getattr(self.plotter.renderer, "renderer", self.plotter.renderer)

    def _qt_pick_xy(self, ev) -> tuple[float, float]:
        pos = ev.position() if hasattr(ev, "position") else ev.pos()
        return float(pos.x()), float(pos.y())

    def _world_to_qt_xy(self, pos) -> Optional[np.ndarray]:
        renderer = self._vtk_renderer()
        renderer.SetWorldPoint(float(pos[0]), float(pos[1]), float(pos[2]), 1.0)
        renderer.WorldToDisplay()
        x, y, _ = renderer.GetDisplayPoint()
        return np.array([x, self.plotter.interactor.height() - y], dtype=np.float64)

    def _nearest_node_at(self, qx: float, qy: float, threshold_px: float = 14.0) -> Optional[int]:
        if not self.model.nodes:
            return None
        mouse = np.array([qx, qy], dtype=np.float64)
        best_id = None
        best_dist = threshold_px
        for nid, node in self.model.nodes.items():
            screen = self._world_to_qt_xy(node["pos"])
            if screen is None:
                continue
            dist = float(np.linalg.norm(screen - mouse))
            if dist <= best_dist:
                best_dist = dist
                best_id = nid
        return best_id

    @staticmethod
    def _point_segment_distance_2d(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
        ab = b - a
        denom = float(np.dot(ab, ab))
        if denom <= 1.0e-9:
            return float(np.linalg.norm(p - a))
        t = float(np.clip(np.dot(p - a, ab) / denom, 0.0, 1.0))
        return float(np.linalg.norm(p - (a + t * ab)))

    def _nearest_edge_at(self, qx: float, qy: float, threshold_px: float = 10.0) -> Optional[int]:
        if not self.model.edges:
            return None
        mouse = np.array([qx, qy], dtype=np.float64)
        best_id = None
        best_dist = threshold_px
        for eid, edge in self.model.edges.items():
            a_node = self.model.nodes.get(edge["from"])
            b_node = self.model.nodes.get(edge["to"])
            if a_node is None or b_node is None:
                continue
            a = self._world_to_qt_xy(a_node["pos"])
            b = self._world_to_qt_xy(b_node["pos"])
            if a is None or b is None:
                continue
            dist = self._point_segment_distance_2d(mouse, a, b)
            if dist <= best_dist:
                best_dist = dist
                best_id = eid
        return best_id

    def _graph_plane_z(self) -> float:
        if not self.model.nodes:
            return 0.0
        return float(np.median([node["pos"][2] for node in self.model.nodes.values()]))

    def _world_on_graph_plane(self, qx: float, qy: float) -> Optional[np.ndarray]:
        renderer = self._vtk_renderer()
        display_y = self.plotter.interactor.height() - qy

        def display_to_world(display_z: float) -> np.ndarray:
            renderer.SetDisplayPoint(qx, display_y, display_z)
            renderer.DisplayToWorld()
            wx, wy, wz, ww = renderer.GetWorldPoint()
            if abs(ww) < 1.0e-9:
                return np.array([wx, wy, wz], dtype=np.float64)
            return np.array([wx / ww, wy / ww, wz / ww], dtype=np.float64)

        near = display_to_world(0.0)
        far = display_to_world(1.0)
        direction = far - near
        if abs(direction[2]) < 1.0e-9:
            return None
        z = self._graph_plane_z()
        t = (z - near[2]) / direction[2]
        return near + t * direction

    def handle_graph_click(self, ev) -> bool:
        if ev.button() != Qt.MouseButton.LeftButton:
            return False
        qx, qy = self._qt_pick_xy(ev)

        if self.mode == self.MODE_VIEW:
            nid = self._nearest_node_at(qx, qy, threshold_px=22.0)
            if nid is not None:
                self.edit_node_metadata(nid)
                return True
            eid = self._nearest_edge_at(qx, qy, threshold_px=12.0)
            if eid is not None:
                self.edit_edge_metadata(eid)
                return True
            return False

        if self.mode == self.MODE_DELETE_NODE:
            nid = self._nearest_node_at(qx, qy)
            if nid is None:
                self._set_status("no node selected for deletion")
                return False
            self.model.remove_node(nid)
            self.redraw_graph()
            self._set_status(f"deleted node {nid}")
            return True

        if self.mode == self.MODE_DELETE_EDGE:
            eid = self._nearest_edge_at(qx, qy)
            if eid is None:
                self._set_status("no edge selected for deletion")
                return False
            self.model.remove_edge(eid)
            self.redraw_graph()
            self._set_status(f"deleted edge {eid}")
            return True

        if self.mode == self.MODE_ADD_NODE:
            pos = self._world_on_graph_plane(qx, qy)
            if pos is None:
                self._set_status("could not place node at this view angle")
                return False
            new_id = self.model.add_node([float(pos[0]), float(pos[1]), float(pos[2])])
            self.redraw_graph()
            self._set_status(f"added node {new_id}")
            return True

        if self.mode == self.MODE_ADD_EDGE:
            nid = self._nearest_node_at(qx, qy, threshold_px=22.0)
            if nid is None:
                self._set_status("pick a node to create an edge")
                return False
            self._on_node_picked(nid)
            return True

        return False

    def handle_metadata_double_click(self, ev) -> bool:
        if self.mode != self.MODE_VIEW:
            return False
        qx, qy = self._qt_pick_xy(ev)

        # Prefer edges first so line segments can be edited even when their
        # endpoints are close on screen.
        eid = self._nearest_edge_at(qx, qy, threshold_px=12.0)
        if eid is not None:
            self.edit_edge_metadata(eid)
            return True

        nid = self._nearest_node_at(qx, qy, threshold_px=14.0)
        if nid is not None:
            self.edit_node_metadata(nid)
            return True

        self._set_status("no node or edge picked for metadata edit")
        return False

    def eventFilter(self, obj, ev):
        if obj is self.plotter.interactor and ev.type() == QEvent.Type.MouseButtonPress:
            self._graph_mouse_press_handled = False
            if ev.button() == Qt.MouseButton.LeftButton:
                qx, qy = self._qt_pick_xy(ev)
                hits_edit_target = (
                    self.mode == self.MODE_VIEW and (
                        self._nearest_node_at(qx, qy, threshold_px=22.0) is not None or
                        self._nearest_edge_at(qx, qy, threshold_px=12.0) is not None
                    )
                )
                if self.mode != self.MODE_VIEW or hits_edit_target:
                    # Prevent VTK camera interaction from starting while graph tools are active.
                    self._graph_mouse_press_handled = True
                    ev.accept()
                    return True
        if obj is self.plotter.interactor and ev.type() == QEvent.Type.MouseMove:
            if self._graph_mouse_press_handled:
                ev.accept()
                return True
        if obj is self.plotter.interactor and ev.type() == QEvent.Type.MouseButtonDblClick:
            if ev.button() == Qt.MouseButton.LeftButton and self.handle_metadata_double_click(ev):
                ev.accept()
                return True
        if obj is self.plotter.interactor and ev.type() == QEvent.Type.MouseButtonRelease:
            if ev.button() != Qt.MouseButton.LeftButton:
                return super().eventFilter(obj, ev)
            handled = self.handle_graph_click(ev)
            if self._graph_mouse_press_handled or handled:
                self._release_camera_interaction()
            if handled:
                ev.accept()
                return True
        if obj is self.plotter.interactor and ev.type() == QEvent.Type.Wheel:
            if ev.modifiers() & Qt.KeyboardModifier.ControlModifier:
                delta = ev.angleDelta().y()
                factor = 1.05 if delta > 0 else 1.0 / 1.05
                self.plotter.camera.Zoom(factor)
                self._refresh_camera_clipping()
                self.plotter.update()
                ev.accept()
                return True
        return super().eventFilter(obj, ev)

    # ── Keyboard ──────────────────────────────────────────────────────────
    def keyPressEvent(self, ev):
        if ev.key() == Qt.Key.Key_Space and self.mode == self.MODE_VIEW:
            self.set_top_down_view()
            ev.accept()
            return
        if ev.key() == Qt.Key.Key_Escape:
            self._edge_drag_source = None
            self._release_camera_interaction()
            self._set_status("selection cleared")
        elif ev.key() == Qt.Key.Key_Delete and self._edge_drag_source is not None:
            self.model.remove_node(self._edge_drag_source)
            self._edge_drag_source = None
            self.redraw_graph()
        super().keyPressEvent(ev)


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print(
            "graph_editor requires a graphical desktop session. "
            "Run it from a terminal inside the Ubuntu desktop, or use SSH with X11 forwarding.",
            file=sys.stderr,
        )
        sys.exit(2)

    ap = argparse.ArgumentParser()
    ap.add_argument("--graph", help="initial graph_3d.json to load")
    ap.add_argument("--pcd",   help="initial PCD file or directory to overlay")
    ap.add_argument(
        "--pcd-downsample-rate",
        type=float,
        default=0.05,
        help="fraction of points kept from each PCD frame before rendering; default: 0.05",
    )
    ap.add_argument(
        "--pcd-max-points",
        type=int,
        default=1000000,
        help="maximum merged PCD points to render; 0 disables the cap; default: 1000000",
    )
    args = ap.parse_args()

    _configure_qt_platform()

    app = QApplication(sys.argv)
    w = GraphEditor(args.pcd_downsample_rate, args.pcd_max_points)
    if args.graph:
        graph_path = Path(args.graph).expanduser()
        if graph_path.exists():
            w.model.load(str(graph_path))
            w.graph_path = str(graph_path)
            w.redraw_graph()
    if args.pcd:
        pcd_path = Path(args.pcd).expanduser()
        if pcd_path.exists():
            w.pcd_path = str(pcd_path)
            w.redraw_pcd()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
