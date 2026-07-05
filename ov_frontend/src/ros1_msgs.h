/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Minimal ros1msg (ROS1 serialization) parsers for the message types the
 * dataset runner consumes. Little-endian, fixed layouts — no ROS required.
 */

#ifndef OV_FRONTEND_ROS1_MSGS_H
#define OV_FRONTEND_ROS1_MSGS_H

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace ov_frontend {

/// Byte cursor over a serialized ros1 message
struct Ros1Cursor {
  const uint8_t *data;
  size_t size;
  size_t off = 0;

  void need(size_t n) const {
    if (off + n > size)
      throw std::runtime_error("ros1msg: truncated message");
  }
  uint32_t u32() {
    need(4);
    uint32_t v;
    std::memcpy(&v, data + off, 4);
    off += 4;
    return v;
  }
  double f64() {
    need(8);
    double v;
    std::memcpy(&v, data + off, 8);
    off += 8;
    return v;
  }
  void skip(size_t n) {
    need(n);
    off += n;
  }
  std::string str() {
    uint32_t n = u32();
    need(n);
    std::string s(reinterpret_cast<const char *>(data + off), n);
    off += n;
    return s;
  }
};

/// std_msgs/Header -> timestamp in seconds (frame_id discarded)
inline double parse_header(Ros1Cursor &c) {
  c.u32(); // seq
  uint32_t sec = c.u32();
  uint32_t nsec = c.u32();
  c.skip(c.u32()); // frame_id
  return sec + nsec * 1e-9;
}

struct Ros1Imu {
  double timestamp;
  double wm[3];
  double am[3];
};

/// sensor_msgs/Imu
inline Ros1Imu parse_imu(const uint8_t *data, size_t size) {
  Ros1Cursor c{data, size};
  Ros1Imu m{};
  m.timestamp = parse_header(c);
  c.skip(4 * 8 + 9 * 8); // orientation + covariance
  for (auto &v : m.wm)
    v = c.f64();
  c.skip(9 * 8); // angular velocity covariance
  for (auto &v : m.am)
    v = c.f64();
  c.skip(9 * 8); // linear acceleration covariance
  return m;
}

struct Ros1CompressedImage {
  double timestamp;
  std::string format;
  const uint8_t *bytes;
  size_t num_bytes;
};

/// sensor_msgs/CompressedImage (bytes point into the source buffer)
inline Ros1CompressedImage parse_compressed_image(const uint8_t *data, size_t size) {
  Ros1Cursor c{data, size};
  Ros1CompressedImage m{};
  m.timestamp = parse_header(c);
  m.format = c.str();
  m.num_bytes = c.u32();
  c.need(m.num_bytes);
  m.bytes = data + c.off;
  return m;
}

struct Ros1PoseStamped {
  double timestamp;
  double p[3];
  double q[4]; // x, y, z, w
};

/// geometry_msgs/PoseStamped
inline Ros1PoseStamped parse_pose_stamped(const uint8_t *data, size_t size) {
  Ros1Cursor c{data, size};
  Ros1PoseStamped m{};
  m.timestamp = parse_header(c);
  for (auto &v : m.p)
    v = c.f64();
  for (auto &v : m.q)
    v = c.f64();
  return m;
}

} // namespace ov_frontend

#endif // OV_FRONTEND_ROS1_MSGS_H
