# viri: vision-based autonomous drone interceptor

Code for *Vision-based Autonomous Drone Interceptor* (Lin & Attafuah, MIE1076). A YOLO26n stereo detector feeds a gravity-aligned LTVKF with rewind-update-replay. The LTVKF drives a ProNav or hybrid heading/ProNav guidance law, which flies the chaser through the RPG autopilot. The full pipeline runs in Gazebo (RotorS physics) with Flightmare (Unity) rendering the camera images.

```
Flightmare stereo ─▶ yolo_stereo_node ─▶ depth_estimation ─▶ relative_ltvkf ─▶ guidance ─▶ autopilot (feedthrough)
   (viri_sim)         (viri_perception)   (viri_perception)  (viri_estimation) (viri_guidance)   (rpg_quadrotor_control)
Gazebo GT odom ─▶ odom_noise_injector ──────────────────────────────▲ prediction @ 100 Hz
```

## Packages

| Package | Contents |
|---|---|
| `viri` | Metapackage |
| `viri_msgs` | `RelativeState` (LTVKF output), `GuidanceStatus` |
| `viri_perception` | `StereoTriangulator` (ROS-free) + `depth_estimation_node`, `yolo_stereo_node.py`, YOLO weights |
| `viri_estimation` | `RelativeLTVKF` (ROS-free) + `relative_ltvkf_node`. Math in [`docs/LTVKF.md`](viri_estimation/docs/LTVKF.md) |
| `viri_guidance` | `ProNav`, `HybridProNav`, attitude mapping (ROS-free) + `guidance_node`. Math in [`docs/GUIDANCE.md`](viri_guidance/docs/GUIDANCE.md) |
| `viri_sim` | Flightmare stereo renderer, odometry noise injector, target trajectories, camera TFs, YOLO data generation |
| `viri_bringup` | Launch files, ground-truth engagement monitor, result plotting |

Every tunable lives in a package's `config/*.yaml`, and the defaults are the paper's values. The stereo rig is defined once, in `viri_sim/config/cameras.yaml`. Both the Unity renderer and the TF tree read it.

## Build and test

```bash
rosdep install --from-paths src --ignore-src -y     # plus the repos in viri/dependencies.yaml
pip install ultralytics
catkin build viri
catkin run_tests viri_perception viri_estimation viri_guidance && catkin_test_results build
```

## Run

```bash
roslaunch viri_bringup interception.launch law:=hybrid trajectory:=hover
#   law:=hybrid|pronav   trajectory:=hover|back_and_forth|figure_eight
#   target_x:=.. target_y:=..   rviz:=false   gui:=false   rqt:=true (manual flight tools)
```

The guidance node does the following on its own:

1. Arms the chaser and requests autopilot take-off.
2. Engages once the autopilot hovers and the LTVKF has initialized from the first stereo detection.
3. Stops commanding when `engagement_monitor` reports a ground-truth intercept (|r| < 1 m). The autopilot then holds hover.

Each run writes a CSV to `~/.ros/viri/`. To get the RMSE table and the paper-style figures:

```bash
rosrun viri_bringup plot_results.py ~/.ros/viri/hybrid_hover_*.csv
```

Manual control is available as `rosservice call /chaser_drone/guidance/engage` and `.../disengage`. Set `auto_engage:=false` to stop the node from engaging on its own.
