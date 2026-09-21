import json
from pathlib import Path
import tempfile
import unittest

from route3d_odom_waypoint.topology_builder import (
    BuilderSettings,
    OdomTopologyBuilder,
    load_pose_file,
)


class PoseFileToTopologyTest(unittest.TestCase):

    def test_pose_file_builds_schema_v2_topology(self):
        with tempfile.TemporaryDirectory() as directory:
            pose_file = Path(directory) / 'pose.json'
            rows = [
                [float(index), index * 0.55, 0.0, 0.0,
                 0.0, 0.0, 0.0, 1.0]
                for index in range(10)
            ]
            pose_file.write_text(
                json.dumps(list(reversed(rows))), encoding='utf-8')

            samples = load_pose_file(pose_file)
            self.assertEqual(
                [sample.stamp for sample in samples],
                [float(index) for index in range(10)])

            builder = OdomTopologyBuilder(BuilderSettings(
                target_spacing=1.0,
                relocation_distance=2.0,
                retrace_enabled=False,
                geometric_loop_closure_enabled=False,
                slope_enabled=False,
                obstacle_mode=2,
            ))
            builder.add_all(samples)
            document = builder.document('camera_init', finalize=True)

            self.assertEqual(
                document['schema'],
                {'name': 'route3d_topology', 'version': 2})
            self.assertEqual(document['frame_id'], 'camera_init')
            self.assertTrue(document['vertices'])
            self.assertTrue(document['edges'])
            self.assertTrue(all(
                edge['meta']['obstacleMode'] == 2
                for edge in document['edges'].values()))
            self.assertTrue(all(
                vertex['alignFinalYaw'] is False
                for vertex in document['vertices'].values()))

    def test_pose_file_requires_two_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            pose_file = Path(directory) / 'pose.json'
            pose_file.write_text(
                json.dumps([
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]),
                encoding='utf-8')

            with self.assertRaisesRegex(ValueError, 'at least two'):
                load_pose_file(pose_file)


if __name__ == '__main__':
    unittest.main()
