# Traversability extraction

This utility extracts a route-supported traversable surface from a registered map and a ROS 2
odometry bag. The bag trajectory provides high-confidence ground seeds; local map normals,
surface continuity, and robot clearance expand and filter the result.

The current defaults are tuned for the `regu` data set: 0.40 m body height, 0.10 m voxels,
30 degree maximum surface slope, and 0.32 m horizontal clearance.

Outputs include the inferred traversable surface, the high-confidence traversed corridor,
candidate surfaces, obstacle points, intermediate diagnostics, and a JSON summary.

## Snap an optimized trajectory onto the mapped surface

When pose-graph optimization leaves a spatially varying Z offset between `pose.json` and the
final PCD, use the candidate surface from the first extraction pass to correct the route.  The
snapper keeps trajectory X/Y, estimates a local map-supported Z correction, and median-filters
that correction along the route so stacked floors do not cause one-frame height jumps.

```bash
python3 tools/traversability_extraction/convert_pose_trajectory.py \
  '/home/wei/map (2)/slam_data/trajectory/pose.json' \
  /tmp/outdoor1_snapped_trajectory \
  --body-height 0.40 \
  --min-spacing 0.04 \
  --snap-surface data/outdoor1/traversability_result/candidate_surface.pcd \
  --snap-xy-radius 0.35 \
  --snap-max-z 0.80 \
  --snap-median-window 51

./tools/traversability_extraction/extract_traversable_region \
  '/home/wei/map (2)/outdoor1.pcd' \
  /tmp/outdoor1_snapped_trajectory/trajectory_ground.pcd \
  /tmp/outdoor1_snapped_result
```

`trajectory_before_snap.pcd` and `trajectory_summary.json` are written beside the snapped
`trajectory_ground.pcd` for comparison and diagnostics.

## Runtime height-range filter

The extractor writes `traversable_surface_unfiltered.pcd` in addition to the filtered and
legacy comparison surfaces.  The planner selects the active surface from
`config/regu_planner.yaml`:

```yaml
height_range_filter.enabled: false
height_range_filter.min_height: 0.40
height_range_filter.max_height: 0.41
```

`false` uses the unfiltered surface and skips height-based obstacle removal.  `true` loads
`filtered_surface_pcd` and applies the configured height band during runtime inflation.
