# OpenArm upstream integration

The two custom ROS 2 packages are independent source packages. The real-hardware
demo additionally needs small changes to the pinned OpenArm upstream sources.
They are published here as patches and new files so this repository does not
copy the full third-party projects.

## Baselines

| Project | Commit |
|---|---|
| `enactic/openarm_description` | `6c7b720f1ba48e8bafa3a3dc752c45f397b42221` |
| `enactic/openarm_ros2` | `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` |

`upstream.repos` is the source of truth for all third-party revisions.

## What the integration changes

`openarm_description.patch`:

- exposes the official gripper grasp helper frame to the robot model;
- carries gravity, gripper contact and simulation arguments into ros2_control;
- adds a small joint4 model tolerance for measured zero/assembly offset;
- selects real, generic fake, or contact-aware simulated hardware plugins.

`openarm_ros2.patch`:

- adds KDL gravity feed-forward to the OpenArm hardware plugin;
- adds gripper torque `disabled`, `monitor`, and `enforce` modes;
- publishes gripper motor torque and contact diagnostics;
- adds contact-aware fake hardware and pure state-machine tests;
- switches the right gripper MoveIt interface to `FollowJointTrajectory`;
- enables command-limit enforcement, explicit acceleration/jerk limits and
  Ruckig smoothing;
- increases right-arm KDL IK timeout from 5 ms to 20 ms;
- moves wrist safety margins into MoveIt's planning bounds;
- enables MTC task-solution execution capability and optional RViz startup.

The files in `openarm_ros2_new_files/` are additions referenced by the tracked
source patch. Copy them into the OpenArm ROS 2 checkout after applying the
patch. The root README contains copy-paste commands.

## Safety and portability

The joint4 lower-bound adjustment and all control gains must be checked against
the user's physical robot. Gravity feed-forward depends on the URDF inertial
model. Torque values are motor-side torque, not fingertip force; `enforce` must
remain disabled until a local monitor-mode calibration is complete.

Do not apply these patches to a different upstream revision without reviewing
the resulting URDF, controller and hardware-interface behavior.
