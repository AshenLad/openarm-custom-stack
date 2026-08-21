# openarm_mtc_pick

MoveIt Task Constructor package for the one-shot RGB-D vertical pick/place demo.

The production entry point is:

```bash
ros2 launch openarm_mtc_pick pick.launch.py
```

It owns perception for one transaction, commits the stable visual box to the
PlanningScene, plans the complete pick/place/return task, rechecks perception,
and immediately executes the selected solution. Development, fake execution,
Gazebo RGB-D and randomized regression entry points are documented in the
[repository README](../../README.md).

Do not run the production entry together with a second detector or
`scene_manager`; duplicate publishers invalidate the visual transaction.
