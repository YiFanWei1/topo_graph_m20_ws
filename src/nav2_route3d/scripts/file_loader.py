from __future__ import annotations

import json
from pathlib import Path
from typing import Any

try:
    from .graph import Pose3D, RouteEdge3D, RouteGraph3D, RouteNode3D
except ImportError:
    from graph import Pose3D, RouteEdge3D, RouteGraph3D, RouteNode3D


def load_pose_rows(path: str | Path) -> list[Pose3D]:
    """Load an 8*n pose JSON file.

    Accepted JSON formats:
    - [[timestamp, tx, ty, tz, qx, qy, qz, qw], ...]
    - {"poses": [[...], ...]}
    """

    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    rows = payload["poses"] if isinstance(payload, dict) else payload
    return [Pose3D.from_row(row) for row in rows]


def load_graph(path: str | Path) -> RouteGraph3D:
    path = Path(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("type") == "FeatureCollection":
        return graph_from_geojson(payload)
    return graph_from_json(payload)


def save_graph_json(graph: RouteGraph3D, path: str | Path) -> None:
    Path(path).write_text(json.dumps(graph_to_json(graph), ensure_ascii=False, indent=2), encoding="utf-8")


def save_graph_geojson(graph: RouteGraph3D, path: str | Path) -> None:
    Path(path).write_text(
        json.dumps(graph_to_geojson(graph), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def graph_to_json(graph: RouteGraph3D) -> dict[str, Any]:
    return {
        "format": "nav_route3d",
        "version": 1,
        "frame_id": graph.frame_id,
        "metadata": graph.metadata,
        "nodes": [
            {
                "id": node.node_id,
                "pose": [
                    node.pose.timestamp,
                    node.pose.tx,
                    node.pose.ty,
                    node.pose.tz,
                    node.pose.qx,
                    node.pose.qy,
                    node.pose.qz,
                    node.pose.qw,
                ],
                "metadata": node.metadata,
                "real_neighbors": sorted(node.real_neighbors),
                "fake_neighbors": sorted(node.fake_neighbors),
            }
            for node in sorted(graph.nodes.values(), key=lambda item: item.node_id)
        ],
        "edges": [
            {
                "id": edge.edge_id,
                "start_id": edge.start_id,
                "end_id": edge.end_id,
                "cost": edge.cost,
                "bidirectional": edge.bidirectional,
                "metadata": edge.metadata,
                "operations": edge.operations,
            }
            for edge in sorted(graph.edges.values(), key=lambda item: item.edge_id)
        ],
    }


def graph_from_json(payload: dict[str, Any]) -> RouteGraph3D:
    graph = RouteGraph3D(
        frame_id=payload.get("frame_id", "map"),
        metadata=payload.get("metadata", {}),
    )
    for item in payload.get("nodes", []):
        node = RouteNode3D(
            node_id=int(item["id"]),
            pose=Pose3D.from_row(item["pose"]),
            metadata=dict(item.get("metadata", {})),
            real_neighbors=set(int(v) for v in item.get("real_neighbors", [])),
            fake_neighbors=set(int(v) for v in item.get("fake_neighbors", [])),
        )
        graph.add_node(node)
    for item in payload.get("edges", []):
        graph.add_edge(
            int(item["start_id"]),
            int(item["end_id"]),
            cost=float(item.get("cost", graph.distance(int(item["start_id"]), int(item["end_id"])))),
            metadata=dict(item.get("metadata", {})),
            operations=list(item.get("operations", [])),
            bidirectional=bool(item.get("bidirectional", True)),
            edge_id=int(item["id"]),
        )
    return graph


def graph_to_geojson(graph: RouteGraph3D) -> dict[str, Any]:
    features: list[dict[str, Any]] = []
    for node in sorted(graph.nodes.values(), key=lambda item: item.node_id):
        features.append(
            {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [node.x, node.y, node.z],
                },
                "properties": {
                    "kind": "node",
                    "id": node.node_id,
                    "timestamp": node.pose.timestamp,
                    "orientation": [node.pose.qx, node.pose.qy, node.pose.qz, node.pose.qw],
                    "metadata": node.metadata,
                    "real_neighbors": sorted(node.real_neighbors),
                    "fake_neighbors": sorted(node.fake_neighbors),
                },
            }
        )
    for edge in sorted(graph.edges.values(), key=lambda item: item.edge_id):
        start = graph.nodes[edge.start_id]
        end = graph.nodes[edge.end_id]
        features.append(
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[start.x, start.y, start.z], [end.x, end.y, end.z]],
                },
                "properties": {
                    "kind": "edge",
                    "id": edge.edge_id,
                    "start_id": edge.start_id,
                    "end_id": edge.end_id,
                    "cost": edge.cost,
                    "bidirectional": edge.bidirectional,
                    "metadata": edge.metadata,
                    "operations": edge.operations,
                },
            }
        )
    return {
        "type": "FeatureCollection",
        "name": "nav_route3d_graph",
        "properties": {"frame_id": graph.frame_id, "metadata": graph.metadata},
        "features": features,
    }


def graph_from_geojson(payload: dict[str, Any]) -> RouteGraph3D:
    props = payload.get("properties", {})
    graph = RouteGraph3D(frame_id=props.get("frame_id", "map"), metadata=props.get("metadata", {}))
    deferred_edges: list[dict[str, Any]] = []
    for feature in payload.get("features", []):
        properties = feature.get("properties", {})
        kind = properties.get("kind")
        if kind == "node":
            x, y, z = _coordinates3(feature["geometry"]["coordinates"])
            qx, qy, qz, qw = properties.get("orientation", [0.0, 0.0, 0.0, 1.0])
            node = RouteNode3D(
                node_id=int(properties["id"]),
                pose=Pose3D(float(properties.get("timestamp", 0.0)), x, y, z, qx, qy, qz, qw),
                metadata=dict(properties.get("metadata", {})),
                real_neighbors=set(int(v) for v in properties.get("real_neighbors", [])),
                fake_neighbors=set(int(v) for v in properties.get("fake_neighbors", [])),
            )
            graph.add_node(node)
        elif kind == "edge":
            deferred_edges.append(properties)
    for item in deferred_edges:
        graph.add_edge(
            int(item["start_id"]),
            int(item["end_id"]),
            cost=float(item.get("cost", 0.0)),
            metadata=dict(item.get("metadata", {})),
            operations=list(item.get("operations", [])),
            bidirectional=bool(item.get("bidirectional", True)),
            edge_id=int(item["id"]),
        )
    return graph


def merge_geojson_metadata(graph: RouteGraph3D, path: str | Path) -> None:
    """Apply GeoJSON feature properties into graph metadata by node/edge id.

    Feature properties may contain:
    - {"kind": "node", "id": 1, "metadata": {...}}
    - {"kind": "edge", "id": 7, "metadata": {...}}
    """

    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    for feature in payload.get("features", []):
        props = feature.get("properties", {})
        metadata = dict(props.get("metadata", {}))
        kind = props.get("kind")
        if kind == "node" and int(props["id"]) in graph.nodes:
            graph.nodes[int(props["id"])].metadata.update(metadata)
        elif kind == "edge" and int(props["id"]) in graph.edges:
            graph.edges[int(props["id"])].metadata.update(metadata)


def _coordinates3(values: list[float]) -> tuple[float, float, float]:
    if len(values) < 2:
        raise ValueError("GeoJSON coordinates must contain at least x/y")
    z = values[2] if len(values) >= 3 else 0.0
    return (float(values[0]), float(values[1]), float(z))
