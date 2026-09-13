# nav2_route3d Visual Validation Checklist

## Launch RViz Validation

```bash
source install/setup.bash
ros2 launch nav2_route3d route3d_visualization.launch.py   graph_filepath:=/home/wei/shenzhen/map/slam_data/trajectory/route3d_graph.geojson
ros2 launch nav2_route3d route3d_visualization.launch.py   graph_filepath:=/home/wei/silou/map/slam_data/route3d_graph.geojson
```

This launch enables visualization and autostarts the lifecycle node. The normal
`route_server_3d.launch.py` keeps `visualization_enabled` false by default to avoid
publisher creation, graph traversal, marker allocation, and RViz-only runtime cost.

## RViz Topics

- `/route_graph_markers`: full 3D graph.
- `/plan`: `nav_msgs/Path` from the latest route result.
- `/route_markers`: latest active route, start, goal, shortcut segments, operation segments.
- `/route_event_markers`: route operation event edges and labels.

## Color Legend

- Cyan points: graph nodes.
- Yellow larger points: shortcut anchor nodes.
- Gray lines: sequential teach-repeat edges.
- Orange lines: shortcut / radius-visibility edges.
- Magenta lines: operation-capable edges.
- Green line: latest computed route.
- Green sphere: route start.
- Red sphere: route goal.
- Yellow event edge and text: triggered route operation.
- Red translucent rejected-neighbor lines: only shown when `publish_rejected_neighbor_markers:=true`.

## Core Route Planning

```bash
ros2 action send_goal /compute_route nav2_msgs/action/ComputeRoute   "{start_id: 0, goal_id: 100, use_start: false, use_poses: false}"
```

Expected: action succeeds with `error_code: 0`; `/plan` appears in RViz; `/route_markers`
shows the same route with start and goal spheres.

## Pose-Based Route Planning

```bash
ros2 action send_goal /compute_route nav2_msgs/action/ComputeRoute   "{use_poses: true, use_start: true, start: {header: {frame_id: map}, pose: {position: {x: 0.257, y: 3.389, z: 0.111}, orientation: {w: 1.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: -0.808, y: 5.347, z: 0.215}, orientation: {w: 1.0}}}}"
```

Expected: nearest graph nodes are selected and the same visual route appears.

## TF-Based Start Pose

Start a test transform in another terminal:

```bash
ros2 run tf2_ros static_transform_publisher 0.257 3.389 0.111 0 0 0 map base_link
```

Then request a route without `use_start`:

```bash
ros2 action send_goal /compute_route nav2_msgs/action/ComputeRoute   "{use_poses: true, use_start: false, goal: {header: {frame_id: map}, pose: {position: {x: -0.808, y: 5.347, z: 0.215}, orientation: {w: 1.0}}}}"
```
ros2 action send_goal /compute_route nav2_msgs/action/ComputeRoute   "{use_poses: true, use_start: false, goal_id: 1000}"
Expected: route starts near the published `base_link` transform.

## Compute And Track Route

```bash
ros2 action send_goal /compute_and_track_route nav2_msgs/action/ComputeAndTrackRoute   "{start_id: 0, goal_id: 1000, use_start: false, use_poses: false}"
```

Expected: action succeeds, feedback contains route/path fields, and RViz updates the
same `/route_markers` and `/plan` displays.

## Route Graph Reload Service

```bash
ros2 service call /route_server/set_route_graph nav2_msgs/srv/SetRouteGraph   "{graph_filepath: /home/wei/silou/map/slam_data/trajectory/route3d_graph.geojson}"
```

Expected: response `success: true`; `/route_graph_markers` refreshes. Use a different
graph path to visually confirm reload behavior.

## Route Operations

Use the installed operation demo graph:

```bash
ros2 launch nav2_route3d route3d_visualization.launch.py   graph_filepath:=$(ros2 pkg prefix nav2_route3d)/share/nav2_route3d/examples/route3d_operation_graph.json
```

Then send:

```bash
ros2 action send_goal /compute_route nav2_msgs/action/ComputeRoute   "{start_id: 0, goal_id: 3, use_start: false, use_poses: false}"
```

Expected: `/route_events` publishes stair / slope event JSON; RViz shows yellow event
edges and labels from `/route_event_markers`.

## Graph Builder Visual Checks

After running `route_graph_builder_3d`, launch the generated graph with RViz and inspect:

- Sequential chain continuity: gray edges should follow odom order.
- Shortcut pruning: orange shortcuts should be sparse and plausible.
- Anchor density: yellow anchors should match `anchor_segment_length`.
- Rejected candidates: relaunch with `publish_rejected_neighbor_markers:=true` only when debugging pruning, because it can create many red lines.

## Performance Mode

```bash
ros2 launch nav2_route3d route_server_3d.launch.py   graph_filepath:=/home/wei/silou/map/slam_data/trajectory/route3d_graph.geojson   visualization_enabled:=false
```

Expected: no `/route_graph_markers`, `/route_markers`, or `/route_event_markers` topics
exist, and no marker data is built.
