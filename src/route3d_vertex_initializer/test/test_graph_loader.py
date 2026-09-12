import json
import math

from route3d_vertex_initializer.graph_loader import load_graph, quaternion_from_rpy


def test_graph_loader(tmp_path):
    path = tmp_path / 'topoGraph_data.json'
    path.write_text(json.dumps({
        'frame_id': 'camera_init',
        'vertices': {
            '7': {'pos': [1.0, 2.0, 3.0], 'rpy': [0.1, -0.2, 0.3]},
        },
    }), encoding='utf-8')
    graph = load_graph(str(path))
    assert graph.frame_id == 'camera_init'
    assert graph.vertices[7].position == (1.0, 2.0, 3.0)
    assert graph.vertices[7].rpy == (0.1, -0.2, 0.3)


def test_quaternion_yaw_only():
    x, y, z, w = quaternion_from_rpy(0.0, 0.0, math.pi / 2.0)
    assert abs(x) < 1e-9
    assert abs(y) < 1e-9
    assert abs(z - math.sqrt(0.5)) < 1e-9
    assert abs(w - math.sqrt(0.5)) < 1e-9
