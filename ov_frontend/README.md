# ov_frontend — ROS-free dataset runner

Plays an MCAP recording through the OpenVINS `VioManager` and writes a
TUM-format trajectory (plus ground truth, if present) for evaluation with
`ov_eval`.

```
./run_dataset <dataset.yaml> [override.mcap]
```

## Dataset = one .mcap + one dataset.yaml

Every dataset, whatever its original form, is converted **once** into an MCAP
file with `ros1msg`-encoded channels:

| channel            | type                          | required |
|--------------------|-------------------------------|----------|
| IMU topic          | `sensor_msgs/Imu`             | yes      |
| camera topic(s)    | `sensor_msgs/CompressedImage` | yes      |
| ground-truth topic | `geometry_msgs/PoseStamped`   | optional |

Converters:
- EuRoC / TUM-VI style `mav0/` folders: `tools/euroc_folder_to_mcap.py`
  (embeds the PNGs as-is, injects mocap ground truth, writes uncompressed
  chunks so the C++ reader needs no lz4/zstd)
- existing ROS1 bags: `mcap convert in.bag out.mcap` (`brew install mcap`)

## dataset.yaml format

```yaml
%YAML:1.0

mcap_path: "../../data/my_dataset.mcap"   # relative to this file, or absolute
estimator_config: "estimator_config.yaml" # OpenVINS estimator config to load
gt_topic: "/gt/pose"                      # "" if the mcap has no ground truth
playback_rate: 0.0                        # 0 = as fast as possible, 1.0 = realtime
output_dir: "results/my_dataset"          # trajectory outputs (relative to cwd)
use_rerun: 1                              # 1 = visualize (needs ENABLE_RERUN build)
rerun_save: ""                            # "" = spawn live viewer, else write .rrd file
```

## Rerun visualization

Built by default (`-DENABLE_RERUN=OFF` for a dependency-free headless build;
building the bundled Arrow needs `CMAKE_POLICY_VERSION_MINIMUM=3.5` in the
environment with CMake 4). With `use_rerun: 1` the runner logs:

- `world/est` (moving pose transform) + `world/est_path` — trajectory (blue)
- `world/est/cam<N>` — camera `Pinhole` frustum(s), parented to the estimate
  pose through the IMU→camera extrinsic, so each rides with the body
- `world/gt` (moving pose transform) + `world/gt_path` — ground truth (grey).
  The first gt sample is pinned onto the concurrent estimate pose — a fixed
  rigid SE(3) offset (cf. ov_eval `align_se3_single`) applied to every later gt
  sample — because gt often starts after init in a different world frame. This
  makes the overlay show actual drift instead of the frame mismatch; the
  exported `traj_gt.txt` stays raw for ov_eval.

  The paths are logged as siblings of the pose entities, directly under the
  identity `world` frame — not as children `world/est/path`. Rerun inherits an
  entity's `Transform3D` to its descendants, so a path nested under the moving
  `world/est` pose would be re-transformed every frame and appear to drift. The
  `world` frame is declared `RIGHT_HAND_Z_UP` (the VIO world is gravity-aligned
  z-up; Rerun's 3D default is y-up).
- `world/feats/slam` (red) and `world/feats/msckf` (green) — 3D features
- `tracks` — camera frames with drawn feature tracks
- `plots/gyro`, `plots/accel`, `plots/timing/update_ms` — time series

`rerun_save` writes a `.rrd` instead (open later with `rerun file.rrd`) —
useful for headless/CI runs.

Sensor topics are **not** listed here: the runner reads the `rostopic` key of
`imu0`/`cam0..N` from the kalibr chain files referenced by the estimator
config, so the calibration files remain the single source of truth for the
sensor setup. (`rostopic` is the Kalibr key name — it identifies the channel
in the mcap; no ROS involved.)

## Outputs

- `<output_dir>/traj_est.txt` — estimated trajectory, TUM format
- `<output_dir>/traj_gt.txt` — ground truth exported from the mcap

Evaluate with, e.g.:

```
./ov_eval/error_singlerun posyaw traj_gt.txt traj_est.txt
```

## Tuning notes for low-rate / always-moving datasets

Learned on TUM-VI `dataset-calib-cam1_1024_16` (stereo fisheye, 4 Hz images,
gentle continuous motion, never static):

- `init_imu_thresh: 0.0` disables the static initializer entirely. Necessary
  when the platform is never still: otherwise a momentarily-calm window can
  trigger a static init with assumed zero velocity, which poisons the filter.
- `init_max_disparity` low (e.g. 4.0) so gentle motion counts as "moving" and
  attempts route to the dynamic initializer.
- `init_window_time` short (e.g. 1.25 s) at low frame rates: long windows lose
  too many KLT tracks and the dynamic initializer's feature gate
  (`0.75 * init_max_features` surviving features) never passes.
- `init_dyn_num_pose` must fit inside the window (frames = rate x window).
