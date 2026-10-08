To Do

colcon build --packages-select fast_lio --symlink-install

The build requires `patch` (`sudo apt install patch`). CMake applies the
ikd-tree neighbor/voxel tie fix to a build-directory copy; the upstream
submodule is not modified. A patch mismatch stops configuration explicitly.

## Loop closure

Loop closure has three user parameters:

```yaml
loop_closure:
  enable: true
  profile: "balanced"  # balanced or simulation
```

When enabled, place recognition and RViz markers are always active. Detection
has no elapsed-time requirement; keyframe separation only prevents matching a
scan against its immediate neighbors. Markers are published on
`/uwfl2_lc/markers`.

Every advanced value used by each profile is available in
`config/loop_closure/balanced.yaml` and
`config/loop_closure/simulation.yaml`. These files are loaded directly by the
node, so editing the selected file changes the corresponding LC behavior.
