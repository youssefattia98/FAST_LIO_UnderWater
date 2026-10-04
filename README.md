To Do

```bash
colcon build --packages-select fast_lio --symlink-install
```

Live sensors use the system ROS clock:

```bash
ros2 launch fast_lio mapping.launch.py use_sim_time:=false
```

For bag replay, publish the recorded receive clock so timestamp-fault recovery
can compare device headers against a common timeline:

```bash
ros2 launch fast_lio mapping.launch.py use_sim_time:=true
ros2 bag play /path/to/bag --clock
```

All live sensor drivers must stamp messages from the same synchronized ROS
clock. The estimator can reject an impossible future timestamp and can avoid
integrating a stale IMU sample across a reconnect, but it cannot reconstruct
vehicle motion during an interval with no translational observations.

`filter_size_map` sets the compact real-time ikd-tree resolution.
`publish.corrected_map_voxel_size` separately sets the density of
`/uwfl2/corrected_map` and the PCD written by `/map_save`; changing the latter
does not change scan matching.
