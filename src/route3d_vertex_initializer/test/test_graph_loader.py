import json
import math
import tempfile
import unittest
from pathlib import Path

from route3d_vertex_initializer.graph_loader import load_graph, quaternion_from_rpy


class GraphLoaderTest(unittest.TestCase):

    def test_graph_loader(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'topoGraph_data.json'
            path.write_text(json.dumps({
                'frame_id': 'camera_init',
                'vertices': {
                    '7': {'pos': [1.0, 2.0, 3.0], 'rpy': [0.1, -0.2, 0.3]},
                },
            }), encoding='utf-8')
            graph = load_graph(str(path))
        self.assertEqual(graph.frame_id, 'camera_init')
        self.assertEqual(graph.vertices[7].position, (1.0, 2.0, 3.0))
        self.assertEqual(graph.vertices[7].rpy, (0.1, -0.2, 0.3))

    def test_quaternion_yaw_only(self):
        x, y, z, w = quaternion_from_rpy(0.0, 0.0, math.pi / 2.0)
        self.assertLess(abs(x), 1e-9)
        self.assertLess(abs(y), 1e-9)
        self.assertLess(abs(z - math.sqrt(0.5)), 1e-9)
        self.assertLess(abs(w - math.sqrt(0.5)), 1e-9)


if __name__ == '__main__':
    unittest.main()
