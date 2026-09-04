import importlib.machinery
import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "nav2_to_topo_single"
LOADER = importlib.machinery.SourceFileLoader("nav2_to_topo_single", str(SCRIPT))
SPEC = importlib.util.spec_from_loader(LOADER.name, LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
LOADER.exec_module(MODULE)


def test_arbitrary_topology_edges_and_edge_id_gaps_are_preserved(tmp_path):
    topology = tmp_path / "topoGraph_data.json"
    topology.write_text("""{
      "frame_id": "camera_init",
      "topology_mode": "incremental_forest",
      "vertices": {
        "1": {"pos": [0, 0, 0], "rpy": [0, 0, 0], "meta": {"sourceStamp": 1}},
        "2": {"pos": [1, 0, 0], "rpy": [0, 0, 0], "meta": {"sourceStamp": 2}},
        "6": {"pos": [-1, 0, 0], "rpy": [0, 0, 3.14], "meta": {"sourceStamp": 6}}
      },
      "edges": {
        "1": {"v": [1, 2], "weight": 1, "meta": {"dir": 0, "source": "discovery"}},
        "9": {"v": [1, 6], "weight": 1, "meta": {"dir": 0, "source": "edge_split"}}
      }
    }""", encoding="utf-8")

    converted = MODULE.build_nav2_from_topology(topology, 0.4)
    assert [edge["id"] for edge in converted["edges"]] == [1, 9]
    assert {(edge["start_id"], edge["end_id"]) for edge in converted["edges"]} == {
        (1, 2), (1, 6)}
    nodes = {node["id"]: node for node in converted["nodes"]}
    assert nodes[1]["real_neighbors"] == [2, 6]
    assert nodes[6]["real_neighbors"] == [1]
