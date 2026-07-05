/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 *
 * Dataset runner: plays an MCAP recording (ros1msg encoding) through the
 * VioManager. Sensor topics come from the kalibr chain files referenced by
 * the estimator config; runner options come from a small dataset yaml.
 *
 *   ./run_dataset <dataset.yaml> [override.mcap]
 */

#include <chrono>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <thread>

#include <mcap/reader.hpp>
#include <opencv2/opencv.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "rerun_viz.h"
#include "ros1_msgs.h"
#include "state/State.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;
namespace fs = std::filesystem;

std::shared_ptr<VioManager> sys;

void signal_callback_handler(int signum) { std::exit(signum); }

struct TrajPoint {
  double t;
  double p[3];
  double q[4]; // x y z w
};

static void write_tum(const std::string &path, const std::vector<TrajPoint> &traj) {
  std::ofstream f(path);
  f << "# timestamp(s) tx ty tz qx qy qz qw" << std::endl;
  f.precision(9);
  f.setf(std::ios::fixed, std::ios::floatfield);
  for (const auto &p : traj)
    f << p.t << " " << p.p[0] << " " << p.p[1] << " " << p.p[2] << " " << p.q[0] << " " << p.q[1] << " " << p.q[2] << " " << p.q[3]
      << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 2) {
    PRINT_ERROR(RED "usage: %s <dataset.yaml> [override.mcap]\n" RESET, argv[0]);
    return EXIT_FAILURE;
  }
  signal(SIGINT, signal_callback_handler);

  // ------------------------------------------------------------------
  // Load the dataset runner configuration
  // ------------------------------------------------------------------
  fs::path dataset_yaml = fs::absolute(argv[1]);
  cv::FileStorage fsread(dataset_yaml.string(), cv::FileStorage::READ);
  if (!fsread.isOpened()) {
    PRINT_ERROR(RED "unable to open dataset config: %s\n" RESET, dataset_yaml.c_str());
    return EXIT_FAILURE;
  }
  std::string mcap_path, estimator_config, gt_topic, output_dir, rerun_save;
  double playback_rate = 0.0;
  int use_rerun = 0;
  fsread["mcap_path"] >> mcap_path;
  fsread["estimator_config"] >> estimator_config;
  fsread["gt_topic"] >> gt_topic;
  fsread["playback_rate"] >> playback_rate;
  fsread["output_dir"] >> output_dir;
  if (!fsread["use_rerun"].empty())
    fsread["use_rerun"] >> use_rerun;
  if (!fsread["rerun_save"].empty())
    fsread["rerun_save"] >> rerun_save;
  fsread.release();

  fs::path base = dataset_yaml.parent_path();
  fs::path mcap_file = argc > 2 ? fs::path(argv[2]) : (fs::path(mcap_path).is_absolute() ? fs::path(mcap_path) : base / mcap_path);
  fs::path est_config = fs::path(estimator_config).is_absolute() ? fs::path(estimator_config) : base / estimator_config;
  if (output_dir.empty())
    output_dir = "results";

  // ------------------------------------------------------------------
  // Create the VIO system from the estimator config
  // ------------------------------------------------------------------
  auto parser = std::make_shared<ov_core::YamlParser>(est_config.string());
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_pubs = false;
  params.use_multi_threading_subs = false;
  sys = std::make_shared<VioManager>(params);

  // Sensor topics from the kalibr chain files (single source of truth)
  std::string topic_imu = "/imu0";
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  std::map<std::string, int> cam_topics; // topic -> cam id
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    std::string topic = "/cam" + std::to_string(i) + "/image_raw";
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", topic);
    cam_topics[topic] = i;
  }

  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    return EXIT_FAILURE;
  }

  // ------------------------------------------------------------------
  // Open the mcap
  // ------------------------------------------------------------------
  mcap::McapReader reader;
  auto status = reader.open(mcap_file.string());
  if (!status.ok()) {
    PRINT_ERROR(RED "unable to open mcap: %s (%s)\n" RESET, mcap_file.c_str(), status.message.c_str());
    return EXIT_FAILURE;
  }
  PRINT_INFO("loaded mcap: %s\n", mcap_file.c_str());

  // Optional Rerun visualization
#ifdef OV_HAVE_RERUN
  std::unique_ptr<ov_frontend::RerunViz> viz;
  if (use_rerun) {
    viz = std::make_unique<ov_frontend::RerunViz>(rerun_save);
    viz->set_cameras(params);
  }
#else
  if (use_rerun)
    PRINT_WARNING(YELLOW "use_rerun requested but built without ENABLE_RERUN\n" RESET);
#endif

  // ------------------------------------------------------------------
  // Playback state
  // ------------------------------------------------------------------
  size_t num_cams = params.state_options.num_cameras;
  bool stereo_pair = (num_cams == 2);
  std::deque<ov_core::CameraData> camera_queue;
  std::map<int64_t, ov_core::CameraData> pending_pairs; // stamp_ns -> partial stereo measurement
  std::vector<TrajPoint> traj_est, traj_gt;
  double last_imu_time = -1;
  size_t count_imu = 0, count_cam = 0, count_updates = 0;
  double sum_update_ms = 0, max_update_ms = 0;
  double first_init_time = -1;

  // wall-clock pacing (playback_rate > 0)
  auto wall_start = std::chrono::steady_clock::now();
  int64_t log_start_ns = -1;

  // Mask for a camera, resized to the incoming image resolution.
  // (params.masks are stored at the post-downsample size; VioManager pyrDowns
  // the runtime mask together with the image, so they must start out equal.)
  auto make_mask = [&](int cam_id, const cv::Mat &img) {
    if (!params.use_mask)
      return cv::Mat(cv::Mat::zeros(img.rows, img.cols, CV_8UC1));
    cv::Mat mask = params.masks.at(cam_id);
    if (mask.size() != img.size())
      cv::resize(mask, mask, img.size(), 0, 0, cv::INTER_NEAREST);
    return mask;
  };

  auto record_state = [&]() {
    if (!sys->initialized())
      return;
    if (first_init_time < 0)
      first_init_time = sys->get_state()->_timestamp;
    auto state = sys->get_state();
    TrajPoint tp;
    tp.t = state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0);
    Eigen::Matrix<double, 4, 1> q = state->_imu->quat();
    Eigen::Matrix<double, 3, 1> p = state->_imu->pos();
    tp.q[0] = q(0), tp.q[1] = q(1), tp.q[2] = q(2), tp.q[3] = q(3);
    tp.p[0] = p(0), tp.p[1] = p(1), tp.p[2] = p(2);
    traj_est.push_back(tp);
  };

  auto process_camera_queue = [&](double t_imu) {
    double calib_dt = sys->get_state()->_calib_dt_CAMtoIMU->value()(0);
    double timestamp_imu_inC = t_imu - calib_dt;
    while (!camera_queue.empty() && camera_queue.front().timestamp < timestamp_imu_inC) {
      auto rT0 = std::chrono::steady_clock::now();
      sys->feed_measurement_camera(camera_queue.front());
      auto rT1 = std::chrono::steady_clock::now();
      double ms = std::chrono::duration<double, std::milli>(rT1 - rT0).count();
      sum_update_ms += ms;
      max_update_ms = std::max(max_update_ms, ms);
      count_updates++;
      record_state();
#ifdef OV_HAVE_RERUN
      if (viz)
        viz->log_update(camera_queue.front().timestamp, sys, ms);
#endif
      camera_queue.pop_front();
    }
  };

  // ------------------------------------------------------------------
  // Main playback loop (messages are stored in log-time order)
  // ------------------------------------------------------------------
  auto view = reader.readMessages();
  for (const auto &mv : view) {
    const std::string &topic = mv.channel->topic;
    const uint8_t *data = reinterpret_cast<const uint8_t *>(mv.message.data);
    size_t size = mv.message.dataSize;

    // realtime pacing
    if (playback_rate > 0) {
      if (log_start_ns < 0)
        log_start_ns = mv.message.logTime;
      double log_elapsed = (mv.message.logTime - log_start_ns) * 1e-9 / playback_rate;
      double wall_elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
      if (log_elapsed > wall_elapsed)
        std::this_thread::sleep_for(std::chrono::duration<double>(log_elapsed - wall_elapsed));
    }

    if (topic == topic_imu) {
      auto m = ov_frontend::parse_imu(data, size);
      ov_core::ImuData message;
      message.timestamp = m.timestamp;
      message.wm << m.wm[0], m.wm[1], m.wm[2];
      message.am << m.am[0], m.am[1], m.am[2];
      sys->feed_measurement_imu(message);
      last_imu_time = message.timestamp;
      count_imu++;
#ifdef OV_HAVE_RERUN
      if (viz)
        viz->log_imu(message.timestamp, message.wm, message.am);
#endif
      process_camera_queue(last_imu_time);

    } else if (cam_topics.count(topic)) {
      int cam_id = cam_topics.at(topic);
      auto m = ov_frontend::parse_compressed_image(data, size);
      cv::Mat raw(1, (int)m.num_bytes, CV_8UC1, const_cast<uint8_t *>(m.bytes));
      cv::Mat img = cv::imdecode(raw, cv::IMREAD_GRAYSCALE);
      if (img.empty()) {
        PRINT_WARNING(YELLOW "failed to decode image on %s\n" RESET, topic.c_str());
        continue;
      }
      count_cam++;

      if (!stereo_pair) {
        ov_core::CameraData message;
        message.timestamp = m.timestamp;
        message.sensor_ids.push_back(cam_id);
        message.images.push_back(img);
        message.masks.push_back(make_mask(cam_id, img));
        camera_queue.push_back(message);
      } else {
        // pair stereo frames by exact timestamp
        int64_t key = (int64_t)mv.message.logTime;
        auto &pending = pending_pairs[key];
        if (pending.sensor_ids.empty())
          pending.timestamp = m.timestamp;
        pending.sensor_ids.push_back(cam_id);
        pending.images.push_back(img);
        pending.masks.push_back(make_mask(cam_id, img));
        if (pending.sensor_ids.size() == 2) {
          // order by sensor id (cam0 first)
          if (pending.sensor_ids[0] > pending.sensor_ids[1]) {
            std::swap(pending.sensor_ids[0], pending.sensor_ids[1]);
            std::swap(pending.images[0], pending.images[1]);
            std::swap(pending.masks[0], pending.masks[1]);
          }
          camera_queue.push_back(pending);
          pending_pairs.erase(key);
        }
        // drop stale unmatched frames (> 0.5s old)
        while (!pending_pairs.empty() && (key - pending_pairs.begin()->first) * 1e-9 > 0.5) {
          PRINT_WARNING(YELLOW "dropping unmatched stereo frame at %.9f\n" RESET, pending_pairs.begin()->first * 1e-9);
          pending_pairs.erase(pending_pairs.begin());
        }
      }

    } else if (!gt_topic.empty() && topic == gt_topic) {
      auto m = ov_frontend::parse_pose_stamped(data, size);
      TrajPoint tp;
      tp.t = m.timestamp;
      for (int i = 0; i < 3; i++)
        tp.p[i] = m.p[i];
      for (int i = 0; i < 4; i++)
        tp.q[i] = m.q[i];
      traj_gt.push_back(tp);
#ifdef OV_HAVE_RERUN
      if (viz)
        viz->log_gt(m.timestamp, m.p, m.q);
#endif
    }
  }
  reader.close();

  // flush anything still queued using the final imu time
  if (last_imu_time > 0)
    process_camera_queue(last_imu_time + 1.0);

  // ------------------------------------------------------------------
  // Outputs
  // ------------------------------------------------------------------
  fs::create_directories(output_dir);
  fs::path est_path = fs::path(output_dir) / "traj_est.txt";
  write_tum(est_path.string(), traj_est);
  PRINT_INFO(GREEN "wrote %zu estimate poses to %s\n" RESET, traj_est.size(), est_path.c_str());
  if (!traj_gt.empty()) {
    fs::path gt_path = fs::path(output_dir) / "traj_gt.txt";
    write_tum(gt_path.string(), traj_gt);
    PRINT_INFO(GREEN "wrote %zu ground-truth poses to %s\n" RESET, traj_gt.size(), gt_path.c_str());
  }

  PRINT_INFO("=======================================\n");
  PRINT_INFO("dataset playback finished\n");
  PRINT_INFO("  imu messages   : %zu\n", count_imu);
  PRINT_INFO("  camera frames  : %zu\n", count_cam);
  PRINT_INFO("  filter updates : %zu\n", count_updates);
  PRINT_INFO("  update time    : %.2f ms avg / %.2f ms max\n", count_updates ? sum_update_ms / count_updates : 0.0, max_update_ms);
  if (sys->initialized()) {
    auto state = sys->get_state();
    PRINT_INFO("  final position : %.3f %.3f %.3f\n", state->_imu->pos()(0), state->_imu->pos()(1), state->_imu->pos()(2));
  } else {
    PRINT_WARNING(YELLOW "  system never initialized!\n" RESET);
  }
  PRINT_INFO("=======================================\n");

  return EXIT_SUCCESS;
}
