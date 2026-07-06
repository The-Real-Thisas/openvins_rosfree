# OpenVINS ROS-Free

> A personal fork of [OpenVINS](https://github.com/rpng/open_vins) that I put
> together to get it building and running with **no ROS and no Docker**,
> developed and verified natively on **macOS / Apple Silicon**. It's a work in
> progress.

[OpenVINS](https://github.com/rpng/open_vins) is a filter-based visual-inertial
estimator from the [Robot Perception and Navigation Group (RPNG)](https://sites.udel.edu/robot/).
Upstream is built around ROS 1/2, catkin and Docker. This fork strips all of that
out and replaces it with:

- a single **C++17 CMake** tree that builds with plain Homebrew dependencies, and
- **`ov_frontend`** — a small ROS-free runner that plays an
  [MCAP](https://mcap.dev) recording through the estimator, writes a TUM-format
  trajectory for evaluation, and can visualize live in the
  [Rerun](https://rerun.io) viewer.

The estimator internals (`ov_msckf`, `ov_init`, `ov_core`, `ov_eval`) are kept
recognizable so upstream fixes stay easy to pull in.

> **Status:** builds and runs on macOS (Apple Silicon, Apple Clang 17). There is
> no macOS-specific code, but other platforms are untested. Work in progress —
> the build layout and the frontend may still change.

## Dependencies (Homebrew)

```bash
brew install cmake eigen opencv boost ceres-solver
```

Developed against Eigen 5.0, OpenCV 4.13, Boost 1.90 and Ceres 2.2.

Optional extras:

- **Live visualization** — the Rerun C++ SDK is fetched automatically by CMake;
  to open the live viewer you also need the viewer binary on your `PATH`:
  ```bash
  pip install rerun-sdk       # provides the `rerun` viewer
  ```
- **Converting your own datasets** — Python 3 with `pip install mcap` (only
  needed for the folder→MCAP tools in `tools/`).

## Build

```bash
mkdir build && cd build
CMAKE_POLICY_VERSION_MINIMUM=3.5 cmake ..
CMAKE_POLICY_VERSION_MINIMUM=3.5 make -j
```

The `CMAKE_POLICY_VERSION_MINIMUM=3.5` in the environment is only needed because
the Rerun SDK bundles an Arrow build that declares an old minimum CMake policy,
which CMake 4 rejects by default. For a lean headless build with no viewer,
disable Rerun and drop the env var:

```bash
mkdir build && cd build
cmake .. -DENABLE_RERUN=OFF
make -j
```

## Get the datasets

Two ready-to-run datasets are published as MCAP recordings on the
[releases page](https://github.com/The-Real-Thisas/openvins_rosfree/releases/tag/datasets-v1).
Download them into a `data/` folder at the repo root:

```bash
mkdir -p data
gh release download datasets-v1 --repo The-Real-Thisas/openvins_rosfree --dir data
```

or, without the GitHub CLI:

```bash
mkdir -p data
curl -L -o data/tumvi_calib_cam1_1024.mcap \
  https://github.com/The-Real-Thisas/openvins_rosfree/releases/download/datasets-v1/tumvi_calib_cam1_1024.mcap
curl -L -o data/uzh_fpv_indoor_forward_6.mcap \
  https://github.com/The-Real-Thisas/openvins_rosfree/releases/download/datasets-v1/uzh_fpv_indoor_forward_6.mcap
```

| file | source sequence | ground truth |
|------|-----------------|--------------|
| `tumvi_calib_cam1_1024.mcap` | TUM-VI `dataset-calib-cam1_1024_16` (stereo fisheye) | motion capture |
| `uzh_fpv_indoor_forward_6.mcap` | UZH-FPV `indoor_forward_6` (Snapdragon stereo) | Leica total station |

Each `.mcap` bundles the camera and IMU streams together with the ground-truth
trajectory. See [`ov_frontend/README.md`](ov_frontend/README.md) for the recording
format and the folder→MCAP converters under `tools/`.

## Run

From the `build/` directory, point the runner at a dataset config:

```bash
./ov_frontend/run_dataset ../config/tumvi_calib_1024/dataset.yaml
# or
./ov_frontend/run_dataset ../config/uzhfpv_indoor_forward_6/dataset.yaml
```

Each `config/<name>/dataset.yaml` selects the mcap, the estimator config, the
playback rate, and whether to visualize. Set `use_rerun: 1` to open the Rerun
viewer (needs a Rerun build), or `0` to run headless. Trajectory outputs are
written under `build/<output_dir>/` as TUM-format `traj_est.txt` and
`traj_gt.txt`.

Evaluate the estimate against ground truth with `ov_eval`:

```bash
./ov_eval/error_singlerun posyaw \
  results/tumvi_calib_cam1/traj_gt.txt \
  results/tumvi_calib_cam1/traj_est.txt
```

For reference, a posyaw-aligned run lands around **7 mm / 0.5°** on the TUM-VI
calib sequence and **0.19 m / 1.7°** on UZH-FPV `indoor_forward_6`.

## Dataset credits

The bundled recordings are re-packaged into MCAP from two public datasets — if
you use them, please cite the originals:

- **TUM-VI** — D. Schubert, T. Goll, N. Demmel, V. Usenko, J. Stückler and
  D. Cremers, *The TUM VI Benchmark for Evaluating Visual-Inertial Odometry*,
  IROS 2018.
  [dataset page](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset)
- **UZH-FPV** — J. Delmerico, T. Cieslewski, H. Rebecq, M. Faessler and
  D. Scaramuzza, *Are We Ready for Autonomous Drone Racing? The UZH-FPV Drone
  Racing Dataset*, ICRA 2019. [dataset page](https://fpv.ifi.uzh.ch/)

## Credit / Licensing

The underlying OpenVINS code was written by the
[Robot Perception and Navigation Group (RPNG)](https://sites.udel.edu/robot/) at
the University of Delaware. For researchers that have leveraged or compared to
this work, please cite the following:

```txt
@Conference{Geneva2020ICRA,
  Title      = {{OpenVINS}: A Research Platform for Visual-Inertial Estimation},
  Author     = {Patrick Geneva and Kevin Eckenhoff and Woosik Lee and Yulin Yang and Guoquan Huang},
  Booktitle  = {Proc. of the IEEE International Conference on Robotics and Automation},
  Year       = {2020},
  Address    = {Paris, France},
  Url        = {\url{https://github.com/rpng/open_vins}}
}
```

The codebase and documentation is licensed under the [GNU General Public License v3 (GPL-3)](https://www.gnu.org/licenses/gpl-3.0.txt).
You must preserve the copyright and license notices in your derivative work and make available the complete source code with modifications under the same license ([see this](https://choosealicense.com/licenses/gpl-3.0/); this is not legal advice).
