from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Dict, Tuple


@dataclass(frozen=True)
class VertexPose:
    vertex_id: int
    position: Tuple[float, float, float]
    rpy: Tuple[float, float, float]


@dataclass(frozen=True)
class GraphSnapshot:
    path: Path
    frame_id: str
    vertices: Dict[int, VertexPose]


def _finite_triplet(value, field: str, vertex_id: int) -> Tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'vertex {vertex_id}: {field} must contain exactly 3 values')
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f'vertex {vertex_id}: {field} contains a non-finite value')
    return result  # type: ignore[return-value]


def load_graph(path: str) -> GraphSnapshot:
    graph_path = Path(path).expanduser()
    if not graph_path.is_absolute():
        graph_path = graph_path.resolve()
    if not graph_path.is_file():
        raise FileNotFoundError(f'topology graph does not exist: {graph_path}')

    with graph_path.open('r', encoding='utf-8') as stream:
        document = json.load(stream)

    frame_id = str(document.get('frame_id', 'map'))
    raw_vertices = document.get('vertices')
    if raw_vertices is None:
        raise ValueError('topology graph does not contain a vertices field')

    vertices: Dict[int, VertexPose] = {}
    if isinstance(raw_vertices, dict):
        items = raw_vertices.items()
    elif isinstance(raw_vertices, list):
        items = []
        for index, value in enumerate(raw_vertices):
            if not isinstance(value, dict):
                continue
            raw_id = value.get('id', index)
            items.append((raw_id, value))
    else:
        raise ValueError('vertices must be an object or an array')

    for raw_id, vertex in items:
        if not isinstance(vertex, dict):
            continue
        try:
            vertex_id = int(raw_id)
        except (TypeError, ValueError):
            if 'id' not in vertex:
                continue
            vertex_id = int(vertex['id'])
        position = _finite_triplet(vertex.get('pos'), 'pos', vertex_id)
        rpy = _finite_triplet(vertex.get('rpy', [0.0, 0.0, 0.0]), 'rpy', vertex_id)
        vertices[vertex_id] = VertexPose(vertex_id, position, rpy)

    if not vertices:
        raise ValueError('topology graph contains no usable vertices')

    return GraphSnapshot(graph_path, frame_id, vertices)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Return quaternion (x, y, z, w) for intrinsic roll/pitch/yaw."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)
