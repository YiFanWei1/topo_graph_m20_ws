from __future__ import annotations

from dataclasses import dataclass, field
from math import sqrt
from typing import Any, Iterable


Metadata = dict[str, Any]


def metadata_copy(value: Metadata | None) -> Metadata:
    return dict(value) if value else {}


@dataclass(slots=True)
class Pose3D:
    timestamp: float
    tx: float
    ty: float
    tz: float
    qx: float
    qy: float
    qz: float
    qw: float

    @property
    def xyz(self) -> tuple[float, float, float]:
        return (self.tx, self.ty, self.tz)

    @classmethod
    def from_row(cls, row: Iterable[float]) -> "Pose3D":
        values = list(row)
        if len(values) != 8:
            raise ValueError(f"Pose row must have 8 values, got {len(values)}")
        return cls(*[float(v) for v in values])


@dataclass(slots=True)
class RouteNode3D:
    node_id: int
    pose: Pose3D
    metadata: Metadata = field(default_factory=dict)
    real_neighbors: set[int] = field(default_factory=set)
    fake_neighbors: set[int] = field(default_factory=set)

    @property
    def x(self) -> float:
        return self.pose.tx

    @property
    def y(self) -> float:
        return self.pose.ty

    @property
    def z(self) -> float:
        return self.pose.tz


@dataclass(slots=True)
class RouteEdge3D:
    edge_id: int
    start_id: int
    end_id: int
    cost: float
    metadata: Metadata = field(default_factory=dict)
    operations: list[Metadata] = field(default_factory=list)
    bidirectional: bool = True


class RouteGraph3D:
    """Mutable 3D route graph with Nav2 route-server-like concepts."""

    def __init__(self, frame_id: str = "map", metadata: Metadata | None = None) -> None:
        self.frame_id = frame_id
        self.metadata = metadata_copy(metadata)
        self.nodes: dict[int, RouteNode3D] = {}
        self.edges: dict[int, RouteEdge3D] = {}
        self._edge_lookup: dict[tuple[int, int], int] = {}
        self._next_edge_id = 1

    def add_node(self, node: RouteNode3D) -> None:
        if node.node_id in self.nodes:
            raise ValueError(f"Duplicate node id {node.node_id}")
        self.nodes[node.node_id] = node

    def remove_node(self, node_id: int) -> None:
        if node_id not in self.nodes:
            return
        edge_ids = [
            eid for eid, edge in self.edges.items()
            if edge.start_id == node_id or edge.end_id == node_id
        ]
        for edge_id in edge_ids:
            self.remove_edge(edge_id)
        del self.nodes[node_id]
        for node in self.nodes.values():
            node.real_neighbors.discard(node_id)
            node.fake_neighbors.discard(node_id)

    def add_edge(
        self,
        start_id: int,
        end_id: int,
        cost: float | None = None,
        metadata: Metadata | None = None,
        operations: list[Metadata] | None = None,
        bidirectional: bool = True,
        edge_id: int | None = None,
    ) -> int:
        if start_id == end_id:
            raise ValueError("Self edges are not allowed")
        if start_id not in self.nodes or end_id not in self.nodes:
            raise KeyError(f"Missing node for edge {start_id}->{end_id}")
        key = self._canonical_key(start_id, end_id, bidirectional)
        if key in self._edge_lookup:
            return self._edge_lookup[key]

        resolved_edge_id = edge_id if edge_id is not None else self._next_edge_id
        self._next_edge_id = max(self._next_edge_id, resolved_edge_id + 1)
        resolved_cost = self.distance(start_id, end_id) if cost is None else float(cost)
        edge = RouteEdge3D(
            edge_id=resolved_edge_id,
            start_id=start_id,
            end_id=end_id,
            cost=resolved_cost,
            metadata=metadata_copy(metadata),
            operations=list(operations or []),
            bidirectional=bidirectional,
        )
        self.edges[resolved_edge_id] = edge
        self._edge_lookup[key] = resolved_edge_id
        self.nodes[start_id].real_neighbors.add(end_id)
        if bidirectional:
            self.nodes[end_id].real_neighbors.add(start_id)
        return resolved_edge_id

    def remove_edge(self, edge_id: int) -> None:
        edge = self.edges.pop(edge_id, None)
        if edge is None:
            return
        self._edge_lookup.pop(self._canonical_key(edge.start_id, edge.end_id, edge.bidirectional), None)
        self.nodes.get(edge.start_id, RouteNode3D(-1, Pose3D(0, 0, 0, 0, 0, 0, 0, 1))).real_neighbors.discard(edge.end_id)
        if edge.bidirectional and edge.end_id in self.nodes:
            self.nodes[edge.end_id].real_neighbors.discard(edge.start_id)

    def mark_fake_neighbor(self, a: int, b: int) -> None:
        if a in self.nodes and b in self.nodes:
            self.nodes[a].fake_neighbors.add(b)
            self.nodes[b].fake_neighbors.add(a)

    def distance(self, a: int, b: int) -> float:
        na = self.nodes[a]
        nb = self.nodes[b]
        dx = na.x - nb.x
        dy = na.y - nb.y
        dz = na.z - nb.z
        return sqrt(dx * dx + dy * dy + dz * dz)

    def outgoing_edges(self, node_id: int) -> list[RouteEdge3D]:
        result: list[RouteEdge3D] = []
        for edge in self.edges.values():
            if edge.start_id == node_id:
                result.append(edge)
            elif edge.bidirectional and edge.end_id == node_id:
                result.append(
                    RouteEdge3D(
                        edge_id=edge.edge_id,
                        start_id=edge.end_id,
                        end_id=edge.start_id,
                        cost=edge.cost,
                        metadata=edge.metadata,
                        operations=edge.operations,
                        bidirectional=edge.bidirectional,
                    )
                )
        return result

    def nearest_node(self, x: float, y: float, z: float) -> int:
        if not self.nodes:
            raise ValueError("Cannot query nearest node in empty graph")
        best_id = -1
        best_dist = float("inf")
        for node_id, node in self.nodes.items():
            dx = node.x - x
            dy = node.y - y
            dz = node.z - z
            dist = dx * dx + dy * dy + dz * dz
            if dist < best_dist:
                best_id = node_id
                best_dist = dist
        return best_id

    def _canonical_key(self, start_id: int, end_id: int, bidirectional: bool) -> tuple[int, int]:
        if bidirectional:
            return (min(start_id, end_id), max(start_id, end_id))
        return (start_id, end_id)
