# 2026-09-12 Web console navigation additions

This version extends the previous on-demand-live-cloud/module UI build.

## Loop patrol
- Adds two web entries matching the existing workspace scripts:
  - `sh/5_start_loop_patrol.sh`: fixed start/goal A<->B patrol.
  - `sh/7_start_goal_only_loop.sh`: goal-only A<->B patrol; each leg rematches the current robot position.
- The web page accepts A, B, dwell time, and round-trip count (`0` = infinite).
- The backend writes a runtime YAML under `~/.ros/route3d_web_console/cache/loop_patrol/` and passes it to the selected existing script.
- The loop-patrol process/node is included in live status detection and can be stopped from the page.

## Direct point-click navigation
- Entering the Navigation module automatically enables point-click navigation.
- Clicking a topology vertex publishes `/route3d_dijkstra/goal_request` immediately.
- Leaving the Navigation module returns to normal topology selection, so topology editing does not accidentally navigate.
- A toggle button can temporarily disable/re-enable point-click navigation inside the Navigation module.

## Numeric topology editing
- Floating-point form fields use `step="any"`, fixing browser step-mismatch validation on recorded values with many decimals.
- Numeric values are displayed in compact form (up to 6 decimal places, trailing zeroes removed), so users no longer need to manually trim long decimals before saving.
