# scan_cmd_vel_follower

Conservative follower for converting SCAN-Planner local paths into
`geometry_msgs/msg/Twist`.

Default behavior:

- subscribes `/visual_local_trajectory`
- subscribes `/Odometry`
- publishes `/cmd_vel`
- disables lateral motion (`linear.y = 0`)
- publishes zero if path or odom is stale

Start:

```bash
ros2 launch scan_cmd_vel_follower scan_cmd_vel_follower.launch.py
```

For test runs that should not drive the base, override `cmd_topic` at launch.
