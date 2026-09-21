"""Shared RViz marker construction for live and file topology modes."""

from __future__ import annotations

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def color(red: float, green: float, blue: float, alpha: float = 1.0) -> ColorRGBA:
    result = ColorRGBA()
    result.r, result.g, result.b, result.a = red, green, blue, alpha
    return result


VERTEX_COLORS = {
    'normal': color(0.10, 0.55, 1.00),       # 蓝：普通点
    'slope': color(1.00, 0.82, 0.10),        # 黄：坡点
    'corner': color(1.00, 0.15, 0.12),       # 红：拐点
    'junction': color(0.95, 0.15, 0.95),     # 紫：交汇点
}

EDGE_MODE_COLORS = {
    0: color(0.20, 0.55, 1.00),  # 蓝：按控制器配置选择
    1: color(0.10, 0.90, 0.25),  # 绿：Efficient 3D 主动绕障
    2: color(1.00, 0.20, 0.15),  # 红：PID，关闭普通点云停障
    3: color(0.65, 0.65, 0.65),  # 灰：忽略普通障碍语义
    4: color(0.65, 0.20, 1.00),  # 紫：外部栅格控制
}


def _marker(frame_id: str, namespace: str, marker_id: int,
            marker_type: int) -> Marker:
    result = Marker()
    result.header.frame_id = frame_id
    result.ns = namespace
    result.id = marker_id
    result.type = marker_type
    result.action = Marker.ADD
    result.pose.orientation.w = 1.0
    return result


def _point(values, z_offset: float = 0.0) -> Point:
    result = Point()
    result.x = float(values[0])
    result.y = float(values[1])
    result.z = float(values[2]) + z_offset
    return result


def _vertex_kind(vertex: dict) -> str:
    meta = vertex.get('meta', {})
    if bool(meta.get('isJunction', False)):
        return 'junction'
    if bool(meta.get('isCorner', False)):
        return 'corner'
    if bool(meta.get('isSlope', False)):
        return 'slope'
    return 'normal'


def build_result_markers(
    samples, document: dict, frame_id: str, body_height: float, *,
    show_vertex_labels: bool = True, show_edge_labels: bool = True,
    vertex_scale: float = 0.20, label_scale: float = 0.16,
    clear_existing: bool = False,
) -> MarkerArray:
    """Build the same attributed topology markers for live and file modes."""
    result = MarkerArray()
    if clear_existing:
        clear = Marker()
        clear.header.frame_id = frame_id
        clear.action = Marker.DELETEALL
        result.markers.append(clear)

    vertices = {int(key): value for key, value in document['vertices'].items()}

    raw = _marker(frame_id, 'raw_route', 0, Marker.LINE_STRIP)
    raw.scale.x = 0.035
    raw.color = color(0.58, 0.58, 0.58, 0.85)
    raw.points = [
        _point((sample.point[0], sample.point[1],
                sample.point[2] - body_height), 0.02)
        for sample in samples
    ]
    result.markers.append(raw)

    edges = _marker(
        frame_id, 'topology_edges_by_obstacle_mode', 0, Marker.LINE_LIST)
    edges.scale.x = 0.075
    edges.color.a = 1.0
    loop_edges = _marker(
        frame_id, 'geometric_loop_closure_edges', 0, Marker.LINE_LIST)
    loop_edges.scale.x = 0.14
    loop_edges.color = color(0.00, 1.00, 1.00, 1.0)

    for edge_id_text, edge in sorted(
            document['edges'].items(), key=lambda item: int(item[0])):
        endpoints = edge.get('v', [])
        if len(endpoints) != 2 or any(int(item) not in vertices for item in endpoints):
            continue
        first = vertices[int(endpoints[0])]['pos']
        second = vertices[int(endpoints[1])]['pos']
        edge_color = EDGE_MODE_COLORS.get(
            int(edge.get('meta', {}).get('obstacleMode', 0)),
            color(1.0, 1.0, 1.0))
        edges.points.extend([_point(first, 0.08), _point(second, 0.08)])
        edges.colors.extend([edge_color, edge_color])

        source = str(edge.get('meta', {}).get('source', 'unknown'))
        if source == 'loop_closure':
            loop_edges.points.extend([
                _point(first, 0.11), _point(second, 0.11)])

        if show_edge_labels:
            label = _marker(
                frame_id, 'edge_labels', int(edge_id_text), Marker.TEXT_VIEW_FACING)
            label.pose.position = _point((
                0.5 * (float(first[0]) + float(second[0])),
                0.5 * (float(first[1]) + float(second[1])),
                0.5 * (float(first[2]) + float(second[2])),
            ), 0.22)
            label.text = f'E{edge_id_text}'
            label.scale.z = label_scale
            label.color = edge_color
            result.markers.append(label)
    result.markers.extend([edges, loop_edges])

    points = _marker(
        frame_id, 'topology_vertices_by_attribute', 0, Marker.SPHERE_LIST)
    points.scale.x = points.scale.y = points.scale.z = vertex_scale
    points.color.a = 1.0
    for vertex_id, vertex in sorted(vertices.items()):
        kind = _vertex_kind(vertex)
        points.points.append(_point(vertex['pos'], 0.12))
        points.colors.append(VERTEX_COLORS[kind])
        if show_vertex_labels:
            label = _marker(
                frame_id, 'vertex_labels', vertex_id, Marker.TEXT_VIEW_FACING)
            label.pose.position = _point(vertex['pos'], 0.34)
            label.text = f'V{vertex_id}'
            label.scale.z = label_scale
            label.color = VERTEX_COLORS[kind]
            result.markers.append(label)
    result.markers.append(points)

    return result
