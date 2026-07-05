/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Rerun viewer logging for the dataset runner (compiled when OV_HAVE_RERUN).
 */

#ifndef OV_FRONTEND_RERUN_VIZ_H
#define OV_FRONTEND_RERUN_VIZ_H
#ifdef OV_HAVE_RERUN

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <rerun.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

namespace ov_frontend {

class RerunViz {
public:
  /// If save_path is non-empty, log to a .rrd file; otherwise spawn/connect to a viewer.
  explicit RerunViz(const std::string &save_path) : rec("open_vins") {
    rerun::Error err = save_path.empty() ? rec.spawn() : rec.save(save_path);
    if (err.is_err()) {
      PRINT_WARNING("[rerun]: failed to start (%s), visualization disabled\n", err.description.c_str());
      ok = false;
      return;
    }
    // The VIO world frame is gravity-aligned and right-handed with +Z up, so
    // tell the viewer that (its 3D default is Y-up, which tips the scene onto
    // its side).
    rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
    ok = true;
  }

  /// Log each camera as a Pinhole frustum, parented to the moving `world/est`
  /// body pose via its IMU->camera extrinsic, so the frustum rides with the
  /// estimate in the 3D view. Static (the extrinsics/intrinsics do not change).
  void set_cameras(const ov_msckf::VioManagerOptions &params) {
    if (!ok)
      return;
    for (int cid = 0; cid < params.state_options.num_cameras; cid++) {
      if (!params.camera_intrinsics.count(cid) || !params.camera_extrinsics.count(cid))
        continue;
      std::string ent = "world/est/cam" + std::to_string(cid);

      // Camera pose in the IMU/body frame (T_CtoI), same reconstruction as
      // VioManagerOptions: extrinsic stores [q_ItoC(xyzw); p_IinC].
      Eigen::Vector4d q_ItoC = params.camera_extrinsics.at(cid).block(0, 0, 4, 1);
      Eigen::Vector3d p_IinC = params.camera_extrinsics.at(cid).block(4, 0, 3, 1);
      Eigen::Matrix3d R_CtoI = ov_core::quat_2_Rot(q_ItoC).transpose();
      Eigen::Vector3d p_CinI = -R_CtoI * p_IinC;
      Eigen::Quaterniond q_CtoI(R_CtoI);
      q_CtoI.normalize();
      rec.log_static(ent, rerun::Transform3D::from_translation_rotation(
                              rerun::components::Translation3D((float)p_CinI.x(), (float)p_CinI.y(), (float)p_CinI.z()),
                              rerun::Quaternion::from_xyzw((float)q_CtoI.x(), (float)q_CtoI.y(), (float)q_CtoI.z(), (float)q_CtoI.w())));

      // Intrinsics: focal length + resolution (principal point assumed centred,
      // which is fine for drawing the frustum).
      auto cam = params.camera_intrinsics.at(cid);
      Eigen::MatrixXd val = cam->get_value();
      float fx = (float)val(0, 0), fy = (float)val(1, 0);
      rec.log_static(ent, rerun::Pinhole::from_focal_length_and_resolution({fx, fy}, {(float)cam->w(), (float)cam->h()})
                              .with_image_plane_distance(0.4f));
    }
  }

  void log_imu(double t, const Eigen::Vector3d &wm, const Eigen::Vector3d &am) {
    if (!ok)
      return;
    rec.set_time_timestamp_secs_since_epoch("sensor_time", t);
    rec.log("plots/gyro/x", rerun::Scalars(wm(0)));
    rec.log("plots/gyro/y", rerun::Scalars(wm(1)));
    rec.log("plots/gyro/z", rerun::Scalars(wm(2)));
    rec.log("plots/accel/x", rerun::Scalars(am(0)));
    rec.log("plots/accel/y", rerun::Scalars(am(1)));
    rec.log("plots/accel/z", rerun::Scalars(am(2)));
  }

  void log_gt(double t, const double p[3], const double q[4]) {
    if (!ok)
      return;
    rec.set_time_timestamp_secs_since_epoch("sensor_time", t);
    Eigen::Vector3d p_gt(p[0], p[1], p[2]);
    Eigen::Quaterniond q_gt(q[3], q[0], q[1], q[2]); // (w,x,y,z)
    q_gt.normalize();

    // Ground truth typically starts well after initialization and lives in a
    // different world frame (mocap/Leica), so its raw trajectory sits at a
    // large fixed offset from the estimate. Pin the first gt sample onto the
    // concurrent estimate pose -- a single rigid SE(3) offset (cf. ov_eval's
    // align_se3_single) stored once and applied to every later gt sample -- so
    // the overlay shows the actual drift, not the frame mismatch. Viewer only:
    // the exported traj_gt.txt stays raw for ov_eval.
    if (!gt_aligned && have_est) {
      R_align = est_q.toRotationMatrix() * q_gt.toRotationMatrix().transpose();
      t_align = est_p - R_align * p_gt;
      gt_aligned = true;
      PRINT_INFO("[rerun]: pinned ground truth to estimate at first gt sample (t=%.3f)\n", t);
    }
    Eigen::Vector3d pa = gt_aligned ? Eigen::Vector3d(R_align * p_gt + t_align) : p_gt;
    Eigen::Quaterniond qa = gt_aligned ? Eigen::Quaterniond(R_align * q_gt.toRotationMatrix()) : q_gt;
    qa.normalize();

    // world-space path -> sibling of `world/gt` under the identity `world`
    // frame, so the moving `world/gt` transform is not inherited (would drag
    // the whole strip around each frame).
    path_gt.push_back(rerun::Position3D((float)pa.x(), (float)pa.y(), (float)pa.z()));
    // downsample the strip re-log (mocap is ~120 Hz)
    if (path_gt.size() % 10 == 0)
      rec.log("world/gt_path", rerun::LineStrips3D(rerun::components::LineStrip3D(path_gt))
                                   .with_colors(rerun::Color(120, 120, 120)));
    rec.log("world/gt", rerun::Transform3D::from_translation_rotation(
                            rerun::components::Translation3D((float)pa.x(), (float)pa.y(), (float)pa.z()),
                            rerun::Quaternion::from_xyzw((float)qa.x(), (float)qa.y(), (float)qa.z(), (float)qa.w())));
  }

  void log_update(double t, std::shared_ptr<ov_msckf::VioManager> sys, double update_ms) {
    if (!ok)
      return;
    rec.set_time_timestamp_secs_since_epoch("sensor_time", t);
    rec.log("plots/timing/update_ms", rerun::Scalars(update_ms));
    if (!sys->initialized())
      return;

    // current pose (JPL q_GtoI xyzw == Hamilton q_ItoG xyzw)
    auto state = sys->get_state();
    Eigen::Matrix<double, 4, 1> q = state->_imu->quat();
    Eigen::Matrix<double, 3, 1> p = state->_imu->pos();
    rec.log("world/est", rerun::Transform3D::from_translation_rotation(
                             rerun::components::Translation3D((float)p(0), (float)p(1), (float)p(2)),
                             rerun::Quaternion::from_xyzw((float)q(0), (float)q(1), (float)q(2), (float)q(3))));

    // remember the latest estimate pose so the first gt sample can be pinned to it
    est_p = p;
    est_q = Eigen::Quaterniond(q(3), q(0), q(1), q(2));
    est_q.normalize();
    have_est = true;

    // world-space path -> sibling of `world/est` under the identity `world`
    // frame, so the moving `world/est` transform is not inherited (see log_gt).
    path_est.push_back(rerun::Position3D((float)p(0), (float)p(1), (float)p(2)));
    rec.log("world/est_path",
            rerun::LineStrips3D(rerun::components::LineStrip3D(path_est)).with_colors(rerun::Color(46, 133, 222)));

    // point features currently used by the filter
    auto to_positions = [](const std::vector<Eigen::Vector3d> &feats) {
      std::vector<rerun::Position3D> out;
      out.reserve(feats.size());
      for (const auto &f : feats)
        out.emplace_back((float)f(0), (float)f(1), (float)f(2));
      return out;
    };
    rec.log("world/feats/slam", rerun::Points3D(to_positions(sys->get_features_SLAM()))
                                    .with_colors(rerun::Color(230, 76, 76))
                                    .with_radii(0.008f));
    rec.log("world/feats/msckf", rerun::Points3D(to_positions(sys->get_good_features_MSCKF()))
                                     .with_colors(rerun::Color(76, 200, 120))
                                     .with_radii(0.006f));

    // tracker debug image (feature tracks drawn on the camera frames)
    cv::Mat img = sys->get_historical_viz_image();
    if (!img.empty()) {
      cv::Mat rgb;
      cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);
      std::vector<uint8_t> pixels(rgb.data, rgb.data + rgb.total() * 3);
      rec.log("tracks", rerun::Image::from_rgb24(std::move(pixels),
                                                 rerun::WidthHeight((uint32_t)rgb.cols, (uint32_t)rgb.rows)));
    }
  }

private:
  rerun::RecordingStream rec;
  bool ok = false;
  std::vector<rerun::Position3D> path_est, path_gt;

  // latest estimate pose, used to pin ground truth on its first sample
  Eigen::Vector3d est_p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond est_q = Eigen::Quaterniond::Identity();
  bool have_est = false;
  // fixed gt-world -> est-world rigid offset, computed once at the first gt sample
  bool gt_aligned = false;
  Eigen::Matrix3d R_align = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_align = Eigen::Vector3d::Zero();
};

} // namespace ov_frontend

#endif // OV_HAVE_RERUN
#endif // OV_FRONTEND_RERUN_VIZ_H
