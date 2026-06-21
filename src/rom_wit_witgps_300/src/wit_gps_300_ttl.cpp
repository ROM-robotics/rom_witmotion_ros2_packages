// wit_gps_300_ttl.cpp
//
// ROS 2 driver node for the WitMotion WTGPS-300 (TTL) GNSS module.
//
// The module outputs standard NMEA-0183 sentences over a TTL/UART serial
// link (default 9600 baud). This node reads the serial stream, parses the
// sentences with the `minmea` library and republishes the data as standard
// ROS 2 messages:
//
//   * sensor_msgs/NavSatFix        -> latitude / longitude / altitude / status
//   * geometry_msgs/TwistStamped   -> ground velocity (ENU) from RMC/VTG
//   * sensor_msgs/TimeReference    -> GNSS UTC time
//   * rom_wit_witgps_300/WitGpsStatus -> fix type, DOP and satellites in view
//
// Build with -DROM_DEBUG=ON to print every decoded field via RCLCPP_INFO.

#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "rom_wit_witgps_300/msg/satellite_info.hpp"
#include "rom_wit_witgps_300/msg/wit_gps_status.hpp"

extern "C" {
// minmea is C code: its union uses an anonymous struct, which ISO C++ rejects
// under -Wpedantic. Silence it just for this third-party header.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "minmea.h"
#pragma GCC diagnostic pop
}

namespace
{
constexpr double kKnotsToMps = 0.514444;   // 1 knot in m/s
constexpr double kKphToMps = 1.0 / 3.6;    // 1 km/h in m/s
// Some WTGPS-300 firmwares emit high-precision and proprietary sentences that
// exceed the 80-char NMEA-0183 limit (e.g. $GPATT). Size the line buffer with
// generous headroom; minmea itself imposes no length limit when parsing.
constexpr size_t kMaxLineLength = 512;

// Translate an integer baud rate into the matching termios speed constant.
bool baudToSpeed(int baud, speed_t & speed)
{
  switch (baud) {
    case 4800:   speed = B4800;   return true;
    case 9600:   speed = B9600;   return true;
    case 19200:  speed = B19200;  return true;
    case 38400:  speed = B38400;  return true;
    case 57600:  speed = B57600;  return true;
    case 115200: speed = B115200; return true;
    case 230400: speed = B230400; return true;
    default:     return false;
  }
}
}  // namespace

class WitGps300Ttl : public rclcpp::Node
{
public:
  WitGps300Ttl()
  : rclcpp::Node("wit_gps_300_ttl")
  {
    // --- Parameters (overridable via witgps.yaml) ---------------------------
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 9600);
    frame_id_ = declare_parameter<std::string>("frame_id", "gps");

    fix_topic_ = declare_parameter<std::string>("fix_topic", "gps/fix");
    vel_topic_ = declare_parameter<std::string>("vel_topic", "gps/vel");
    time_ref_topic_ =
      declare_parameter<std::string>("time_ref_topic", "gps/time_reference");
    status_topic_ = declare_parameter<std::string>("status_topic", "gps/status");

    time_ref_source_ =
      declare_parameter<std::string>("time_ref_source", "gps");
    horizontal_accuracy_ =
      declare_parameter<double>("horizontal_accuracy", 2.5);
    strict_checksum_ = declare_parameter<bool>("strict_checksum", true);
    use_vtg_velocity_ = declare_parameter<bool>("use_vtg_velocity", true);
    service_gps_ = declare_parameter<bool>("service_gps", true);
    service_beidou_ = declare_parameter<bool>("service_beidou", true);

    // --- Publishers ---------------------------------------------------------
    fix_pub_ =
      create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic_, 10);
    vel_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(vel_topic_, 10);
    time_pub_ =
      create_publisher<sensor_msgs::msg::TimeReference>(time_ref_topic_, 10);
    status_pub_ =
      create_publisher<rom_wit_witgps_300::msg::WitGpsStatus>(status_topic_, 10);

    if (!openSerial()) {
      RCLCPP_FATAL(
        get_logger(), "Failed to open serial port '%s'. Node will keep retrying.",
        port_.c_str());
    }

    // Poll the serial port at 100 Hz; NMEA cycles are typically 1 Hz so this
    // leaves plenty of headroom for the incoming byte stream.
    read_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&WitGps300Ttl::readSerial, this));

    RCLCPP_INFO(
      get_logger(), "wit_gps_300_ttl started on %s @ %d baud (frame_id='%s')",
      port_.c_str(), baudrate_, frame_id_.c_str());
  }

  ~WitGps300Ttl() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

private:
  // ------------------------------------------------------------------------
  // Serial handling
  // ------------------------------------------------------------------------
  bool openSerial()
  {
    speed_t speed;
    if (!baudToSpeed(baudrate_, speed)) {
      RCLCPP_ERROR(get_logger(), "Unsupported baudrate: %d", baudrate_);
      return false;
    }

    fd_ = ::open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(
        get_logger(), "open(%s) failed: %s", port_.c_str(), std::strerror(errno));
      return false;
    }

    termios tio{};
    if (::tcgetattr(fd_, &tio) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    ::cfmakeraw(&tio);
    ::cfsetispeed(&tio, speed);
    ::cfsetospeed(&tio, speed);

    tio.c_cflag |= (CLOCAL | CREAD);   // ignore modem control lines, enable RX
    tio.c_cflag &= ~CSTOPB;            // 1 stop bit
    tio.c_cflag &= ~CRTSCTS;           // no hardware flow control
    tio.c_cc[VMIN] = 0;                // non-blocking read
    tio.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    ::tcflush(fd_, TCIFLUSH);
    return true;
  }

  void readSerial()
  {
    if (fd_ < 0) {
      // Attempt to (re)open the port on the next cycle.
      if (!openSerial()) {
        return;
      }
    }

    char buf[256];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      RCLCPP_WARN(get_logger(), "read error: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return;
    }
    if (n == 0) {
      return;
    }

    for (ssize_t i = 0; i < n; ++i) {
      char c = buf[i];
      if (c == '\n' || c == '\r') {
        if (!line_.empty()) {
          handleSentence(line_);
          line_.clear();
        }
      } else if (line_.size() < kMaxLineLength) {
        line_.push_back(c);
      } else {
        // Overlong / corrupt line; drop it and resynchronise.
        line_.clear();
      }
    }
  }

  // ------------------------------------------------------------------------
  // NMEA sentence dispatch
  // ------------------------------------------------------------------------
  void handleSentence(const std::string & sentence)
  {
    switch (minmea_sentence_id(sentence.c_str(), strict_checksum_)) {
      case MINMEA_SENTENCE_GGA: handleGga(sentence); break;
      case MINMEA_SENTENCE_RMC: handleRmc(sentence); break;
      case MINMEA_SENTENCE_VTG: handleVtg(sentence); break;
      case MINMEA_SENTENCE_GSA: handleGsa(sentence); break;
      case MINMEA_SENTENCE_GSV: handleGsv(sentence); break;
      default: break;
    }
  }

  void handleGga(const std::string & sentence)
  {
    minmea_sentence_gga f;
    if (!minmea_parse_gga(&f, sentence.c_str())) {
      return;
    }

    last_satellites_used_ = f.satellites_tracked;
    last_hdop_ = minmea_tofloat(&f.hdop);

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = now();
    fix.header.frame_id = frame_id_;

    // Service mask: WTGPS-300 tracks GPS + BeiDou.
    fix.status.service = 0;
    if (service_gps_) {
      fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    }
    if (service_beidou_) {
      fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
    }

    switch (f.fix_quality) {
      case 0:
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
      case 2:
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        break;
      case 4:
      case 5:
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        break;
      default:
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        break;
    }

    fix.latitude = minmea_tocoord(&f.latitude);
    fix.longitude = minmea_tocoord(&f.longitude);
    fix.altitude = minmea_tofloat(&f.altitude);

    // Approximate the covariance from HDOP and a configurable per-axis sigma.
    double hdop = std::isnan(last_hdop_) ? 1.0 : last_hdop_;
    double var = std::pow(hdop * horizontal_accuracy_, 2.0);
    fix.position_covariance[0] = var;
    fix.position_covariance[4] = var;
    fix.position_covariance[8] = std::pow(2.0 * hdop * horizontal_accuracy_, 2.0);
    fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    fix_pub_->publish(fix);

#ifdef ROM_DEBUG
    RCLCPP_INFO(
      get_logger(),
      "[GGA] lat=%.7f lon=%.7f alt=%.2fm fix_quality=%d sats_used=%d hdop=%.2f",
      fix.latitude, fix.longitude, fix.altitude, f.fix_quality,
      f.satellites_tracked, hdop);
#endif
  }

  void handleRmc(const std::string & sentence)
  {
    minmea_sentence_rmc f;
    if (!minmea_parse_rmc(&f, sentence.c_str())) {
      return;
    }

    // Time reference from the RMC date + time.
    timespec ts{};
    if (minmea_gettime(&ts, &f.date, &f.time) == 0) {
      sensor_msgs::msg::TimeReference tref;
      tref.header.stamp = now();
      tref.header.frame_id = frame_id_;
      tref.time_ref.sec = static_cast<int32_t>(ts.tv_sec);
      tref.time_ref.nanosec = static_cast<uint32_t>(ts.tv_nsec);
      tref.source = time_ref_source_;
      time_pub_->publish(tref);
    }

    // Fall back to RMC for velocity when VTG is disabled / unavailable.
    if (!use_vtg_velocity_ && f.valid) {
      double speed_mps = minmea_tofloat(&f.speed) * kKnotsToMps;
      double course_deg = minmea_tofloat(&f.course);
      publishVelocity(speed_mps, course_deg);
    }

#ifdef ROM_DEBUG
    RCLCPP_INFO(
      get_logger(),
      "[RMC] valid=%d lat=%.7f lon=%.7f speed=%.2fkn course=%.1fdeg "
      "date=%02d/%02d/%02d time=%02d:%02d:%02d",
      f.valid, minmea_tocoord(&f.latitude), minmea_tocoord(&f.longitude),
      minmea_tofloat(&f.speed), minmea_tofloat(&f.course),
      f.date.year, f.date.month, f.date.day,
      f.time.hours, f.time.minutes, f.time.seconds);
#endif
  }

  void handleVtg(const std::string & sentence)
  {
    minmea_sentence_vtg f;
    if (!minmea_parse_vtg(&f, sentence.c_str())) {
      return;
    }

    if (use_vtg_velocity_) {
      double speed_mps = minmea_tofloat(&f.speed_kph) * kKphToMps;
      double course_deg = minmea_tofloat(&f.true_track_degrees);
      if (!std::isnan(speed_mps) && !std::isnan(course_deg)) {
        publishVelocity(speed_mps, course_deg);
      }
    }

#ifdef ROM_DEBUG
    RCLCPP_INFO(
      get_logger(), "[VTG] track=%.1fdeg speed=%.2fkm/h",
      minmea_tofloat(&f.true_track_degrees), minmea_tofloat(&f.speed_kph));
#endif
  }

  void handleGsa(const std::string & sentence)
  {
    minmea_sentence_gsa f;
    if (!minmea_parse_gsa(&f, sentence.c_str())) {
      return;
    }

    rom_wit_witgps_300::msg::WitGpsStatus status;
    status.header.stamp = now();
    status.header.frame_id = frame_id_;
    status.fix_type = static_cast<uint8_t>(f.fix_type);
    status.satellites_used = last_satellites_used_;
    status.satellites_visible = last_satellites_visible_;
    status.pdop = minmea_tofloat(&f.pdop);
    status.hdop = minmea_tofloat(&f.hdop);
    status.vdop = minmea_tofloat(&f.vdop);
    status.satellites = sat_list_;
    status_pub_->publish(status);

#ifdef ROM_DEBUG
    RCLCPP_INFO(
      get_logger(),
      "[GSA] fix_type=%d pdop=%.2f hdop=%.2f vdop=%.2f sats_used=%d "
      "sats_visible=%d",
      f.fix_type, status.pdop, status.hdop, status.vdop,
      status.satellites_used, status.satellites_visible);
#endif
  }

  void handleGsv(const std::string & sentence)
  {
    minmea_sentence_gsv f;
    if (!minmea_parse_gsv(&f, sentence.c_str())) {
      return;
    }

    // GSV data arrives split over several sentences (msg_nr of total_msgs).
    // Reset the accumulator on the first sentence of a cycle.
    if (f.msg_nr == 1) {
      sat_accum_.clear();
    }

    for (const auto & s : f.sats) {
      if (s.nr != 0) {
        rom_wit_witgps_300::msg::SatelliteInfo info;
        info.prn = s.nr;
        info.elevation = s.elevation;
        info.azimuth = s.azimuth;
        float snr = minmea_tofloat(&s.snr);
        info.snr = std::isnan(snr) ? -1 : static_cast<int32_t>(snr);
        sat_accum_.push_back(info);
      }
    }

    // Cycle complete: latch the satellite list for the next status message.
    if (f.msg_nr == f.total_msgs) {
      sat_list_ = sat_accum_;
      last_satellites_visible_ = f.total_sats;
    }

#ifdef ROM_DEBUG
    RCLCPP_INFO(
      get_logger(), "[GSV] msg %d/%d total_sats=%d (accumulated=%zu)",
      f.msg_nr, f.total_msgs, f.total_sats, sat_accum_.size());
#endif
  }

  // ------------------------------------------------------------------------
  // Helpers
  // ------------------------------------------------------------------------
  void publishVelocity(double speed_mps, double course_deg)
  {
    double course_rad = course_deg * M_PI / 180.0;
    geometry_msgs::msg::TwistStamped vel;
    vel.header.stamp = now();
    vel.header.frame_id = frame_id_;
    // ENU convention: x = east, y = north.
    vel.twist.linear.x = speed_mps * std::sin(course_rad);
    vel.twist.linear.y = speed_mps * std::cos(course_rad);
    vel.twist.linear.z = 0.0;
    vel_pub_->publish(vel);
  }

  // --- Parameters ---
  std::string port_;
  int baudrate_{9600};
  std::string frame_id_;
  std::string fix_topic_;
  std::string vel_topic_;
  std::string time_ref_topic_;
  std::string status_topic_;
  std::string time_ref_source_;
  double horizontal_accuracy_{2.5};
  bool strict_checksum_{true};
  bool use_vtg_velocity_{true};
  bool service_gps_{true};
  bool service_beidou_{true};

  // --- Serial state ---
  int fd_{-1};
  std::string line_;

  // --- Aggregation state ---
  int last_satellites_used_{0};
  int last_satellites_visible_{0};
  double last_hdop_{1.0};
  std::vector<rom_wit_witgps_300::msg::SatelliteInfo> sat_accum_;
  std::vector<rom_wit_witgps_300::msg::SatelliteInfo> sat_list_;

  // --- ROS interfaces ---
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_pub_;
  rclcpp::Publisher<rom_wit_witgps_300::msg::WitGpsStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WitGps300Ttl>());
  rclcpp::shutdown();
  return 0;
}
