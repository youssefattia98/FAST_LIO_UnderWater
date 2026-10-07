To Do

colcon build --packages-select fast_lio --symlink-install

## Loop closure

Loop closure has three user parameters:

```yaml
loop_closure:
  enable: true
  profile: "balanced"  # balanced, simulation, or sparse_sonar
  diagnostics_directory: ""
```

When enabled, place recognition and RViz markers are always active. Detection
has no elapsed-time requirement; keyframe separation only prevents matching a
scan against its immediate neighbors. Markers are published on
`/uwfl2_lc/markers`.
