"""Shared Route3D topology JSON schema defaults.

The graph container intentionally remains compatible with the existing
``vertices``/``edges`` representation. Navigation-related fields use this
project's own descriptive names so future controllers do not depend on the
legacy graph_pid field vocabulary.
"""

from __future__ import annotations

from typing import Any, Dict, MutableMapping


SCHEMA_NAME = "route3d_topology"
SCHEMA_VERSION = 2


def _object(value: Any, path: str) -> MutableMapping[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be an object")
    return value


def travel_mode_from_direction(direction: int) -> str:
    """Return the readable traversal mode paired with the existing ``dir`` value."""

    return {0: "bidirectional", 1: "first_to_second", 2: "second_to_first"}.get(
        direction, "bidirectional")


def apply_topology_schema(document: Dict[str, Any]) -> Dict[str, Any]:
    """Add schema-v2 navigation defaults without replacing generated metadata.

    The operation is idempotent and mutates ``document`` in place. ``setdefault``
    is deliberate: generated geometry and values edited by a map author always
    take precedence over defaults.
    """

    if not isinstance(document, dict):
        raise ValueError("topology document must be an object")
    vertices = _object(document.get("vertices"), "vertices")
    edges = _object(document.get("edges"), "edges")

    schema = document.setdefault("schema", {})
    schema = _object(schema, "schema")
    schema.setdefault("name", SCHEMA_NAME)
    schema.setdefault("version", SCHEMA_VERSION)
    document.setdefault("sceneMode", "normal")

    for vertex_id, raw_vertex in vertices.items():
        vertex = _object(raw_vertex, f"vertices.{vertex_id}")
        meta = vertex.setdefault("meta", {})
        meta = _object(meta, f"vertices.{vertex_id}.meta")
        meta.setdefault("chargingMode", 0)
        # M20 only stops to align at explicitly marked intermediate vertices.
        # The mission goal is forced to align by the route slicer regardless of
        # this authoring default.
        vertex.setdefault("alignFinalYaw", False)
        is_corner = bool(meta.get("isCorner", False))
        # Intermediate traversal semantics are independent from mission-goal
        # semantics. A normal vertex can therefore be made a mandatory stop/
        # pass point without pretending that it is a geometric corner.
        vertex.setdefault("mustPassThrough", is_corner)
        vertex.setdefault("passRadiusM", 0.28 if is_corner else 0.45)

    for edge_id, raw_edge in edges.items():
        edge = _object(raw_edge, f"edges.{edge_id}")
        meta = edge.setdefault("meta", {})
        meta = _object(meta, f"edges.{edge_id}.meta")
        try:
            direction = int(meta.get("dir", 0))
        except (TypeError, ValueError):
            direction = 0

        edge.setdefault("rotationAllowed", True)
        meta.setdefault("linearSpeedMps", 1.0)
        meta.setdefault("angularSpeedRadps", 0.0)
        meta.setdefault("heightOffsetM", 0.0)
        # New M20 edges use Efficient 3D avoidance by default.
        meta.setdefault("obstacleMode", 0)
        meta.setdefault("travelMode", travel_mode_from_direction(direction))
        meta.setdefault("headingAngleRad", 0.0)
        meta.setdefault("obstacleBoxM", [0.0, 0.0, 0.0, 0.0])
        meta.setdefault("gridMapName", "")
        meta.setdefault("controllerMode", "auto")

    return document
