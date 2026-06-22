// WTGAHRS3 (TTL) ROS 2 driver node
//
// Reads WitMotion register-protocol (0x55 frame) data from a TTL/UART serial
// port using the vendored wit_c_sdk and publishes standard ROS 2 messages:
//   - sensor_msgs/Imu            (acceleration, angular velocity, orientation)
//   - sensor_msgs/MagneticField  (magnetic field)
//   - sensor_msgs/Temperature    (chip temperature)
//   - sensor_msgs/FluidPressure  (barometric pressure)
//   - sensor_msgs/NavSatFix      (GPS latitude / longitude / altitude)
//   - geometry_msgs/TwistStamped (GPS ground speed + heading, ENU)
//   - sensor_msgs/TimeReference  (GNSS UTC time)

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

extern "C"
{
#include "serial.h"
#include "REG.h"
#include "wit_c_sdk.h"
}

// ---- update flags (set from the WIT SDK register callback) -----------------
#define ACC_UPDATE    0x0001
#define GYRO_UPDATE   0x0002
#define ANGLE_UPDATE  0x0004
#define MAG_UPDATE    0x0008
#define PRESS_UPDATE  0x0010
#define GPS_UPDATE    0x0020
#define VEL_UPDATE    0x0040
#define QUAT_UPDATE   0x0080
#define DOP_UPDATE    0x0100
#define TIME_UPDATE   0x0200
#define READ_UPDATE   0x8000

static const int c_uiBaud[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

static int s_fd = -1;
static volatile uint32_t s_cDataUpdate = 0;

// ---- helpers to combine two 16-bit registers into a signed 32-bit value ----
static inline int32_t Reg32(int regLow, int regHigh)
{
    return (static_cast<int32_t>(static_cast<uint16_t>(sReg[regHigh])) << 16) |
           static_cast<uint16_t>(sReg[regLow]);
}

// ---- WIT SDK glue ----------------------------------------------------------
static void WitSerialWriteCb(uint8_t *data, uint32_t len)
{
    if (s_fd >= 0)
    {
        serial_write_data(s_fd, data, static_cast<int>(len));
    }
}

static void WitDelayMsCb(uint16_t ms)
{
    usleep(static_cast<useconds_t>(ms) * 1000);
}

static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum)
{
    for (uint32_t i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
        case MS:        s_cDataUpdate |= TIME_UPDATE;  break;
        case AZ:        s_cDataUpdate |= ACC_UPDATE;   break;
        case GZ:        s_cDataUpdate |= GYRO_UPDATE;  break;
        case HZ:        s_cDataUpdate |= MAG_UPDATE;   break;
        case Yaw:       s_cDataUpdate |= ANGLE_UPDATE; break;
        case HeightH:   s_cDataUpdate |= PRESS_UPDATE; break;
        case LatH:      s_cDataUpdate |= GPS_UPDATE;   break;
        case GPSHeight: s_cDataUpdate |= GPS_UPDATE;   break;
        case GPSVH:     s_cDataUpdate |= VEL_UPDATE;   break;
        case q3:        s_cDataUpdate |= QUAT_UPDATE;  break;
        case VDOP:      s_cDataUpdate |= DOP_UPDATE;   break;
        default:        s_cDataUpdate |= READ_UPDATE;  break;
        }
        uiReg++;
    }
}

static bool AutoScanSensor(rclcpp::Logger logger, const std::string &dev, int &curBaud)
{
    unsigned char buf[1];
    char deviceName[256];
    strncpy(deviceName, dev.c_str(), sizeof(deviceName) - 1);
    deviceName[sizeof(deviceName) - 1] = '\0';

    for (size_t i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++)
    {
        serial_close(s_fd);
        s_fd = serial_open(deviceName, c_uiBaud[i]);
        if (s_fd < 0)
        {
            continue;
        }

        int retry = 2;
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            WitDelayMsCb(200);
            while (serial_read_data(s_fd, buf, 1) > 0)
            {
                WitSerialDataIn(buf[0]);
            }
            if (s_cDataUpdate != 0)
            {
                curBaud = c_uiBaud[i];
                RCLCPP_INFO(logger, "WTGAHRS3 found at %d baud.", c_uiBaud[i]);
                return true;
            }
            retry--;
        } while (retry);
    }

    RCLCPP_ERROR(logger, "Sensor not found. Please check the connection / port.");
    return false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("wit_wtgahrs3_ttl");
    auto logger = node->get_logger();

    // ---- parameters --------------------------------------------------------
    const std::string port = node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const int baudrate = node->declare_parameter<int>("baudrate", 9600);
    const bool auto_scan = node->declare_parameter<bool>("auto_scan", true);
    const std::string imu_frame = node->declare_parameter<std::string>("imu_frame_id", "imu_link");
    const std::string gps_frame = node->declare_parameter<std::string>("gps_frame_id", "gps");
    const std::string time_ref_source = node->declare_parameter<std::string>("time_ref_source", "gps");
    const double loop_rate_hz = node->declare_parameter<double>("loop_rate", 200.0);

    const std::string imu_topic = node->declare_parameter<std::string>("imu_topic", "imu/data");
    const std::string mag_topic = node->declare_parameter<std::string>("mag_topic", "imu/mag");
    const std::string temp_topic = node->declare_parameter<std::string>("temperature_topic", "imu/temperature");
    const std::string press_topic = node->declare_parameter<std::string>("pressure_topic", "imu/pressure");
    const std::string fix_topic = node->declare_parameter<std::string>("fix_topic", "gps/fix");
    const std::string vel_topic = node->declare_parameter<std::string>("vel_topic", "gps/vel");
    const std::string time_topic = node->declare_parameter<std::string>("time_topic", "gps/time_reference");

    // ---- open serial -------------------------------------------------------
    char deviceName[256];
    strncpy(deviceName, port.c_str(), sizeof(deviceName) - 1);
    deviceName[sizeof(deviceName) - 1] = '\0';

    s_fd = serial_open(deviceName, baudrate);
    if (s_fd < 0)
    {
        RCLCPP_ERROR(logger, "Failed to open serial port %s", port.c_str());
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(logger, "Opened %s @ %d baud.", port.c_str(), baudrate);

    // ---- init WIT SDK ------------------------------------------------------
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(WitSerialWriteCb);
    WitDelayMsRegister(WitDelayMsCb);
    WitRegisterCallBack(SensorDataUpdate);

    int curBaud = baudrate;
    if (auto_scan)
    {
        if (!AutoScanSensor(logger, port, curBaud))
        {
            serial_close(s_fd);
            rclcpp::shutdown();
            return 1;
        }
    }

    // ---- publishers --------------------------------------------------------
    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
    auto mag_pub = node->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, 10);
    auto temp_pub = node->create_publisher<sensor_msgs::msg::Temperature>(temp_topic, 10);
    auto press_pub = node->create_publisher<sensor_msgs::msg::FluidPressure>(press_topic, 10);
    auto fix_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic, 10);
    auto vel_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(vel_topic, 10);
    auto time_pub = node->create_publisher<sensor_msgs::msg::TimeReference>(time_topic, 10);

    // ---- scale constants ---------------------------------------------------
    constexpr double kAccScale = 16.0 / 32768.0 * 9.80665; // -> m/s^2
    constexpr double kGyroScale = 2000.0 / 32768.0 * M_PI / 180.0; // -> rad/s
    constexpr double kQuatScale = 1.0 / 32768.0;
    constexpr double kKmhToMs = 1.0 / 3.6;

    unsigned char buf[256];
    rclcpp::Rate rate(loop_rate_hz);

    RCLCPP_INFO(logger, "WTGAHRS3 driver running.");

    while (rclcpp::ok())
    {
        int n = serial_read_data(s_fd, buf, sizeof(buf));
        for (int i = 0; i < n; i++)
        {
            WitSerialDataIn(buf[i]);
        }

        const rclcpp::Time stamp = node->get_clock()->now();

        // ---- IMU (acc + gyro + orientation) --------------------------------
        if (s_cDataUpdate & (ACC_UPDATE | GYRO_UPDATE | QUAT_UPDATE))
        {
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = stamp;
            imu_msg.header.frame_id = imu_frame;

            imu_msg.linear_acceleration.x = sReg[AX] * kAccScale;
            imu_msg.linear_acceleration.y = sReg[AY] * kAccScale;
            imu_msg.linear_acceleration.z = sReg[AZ] * kAccScale;

            imu_msg.angular_velocity.x = sReg[GX] * kGyroScale;
            imu_msg.angular_velocity.y = sReg[GY] * kGyroScale;
            imu_msg.angular_velocity.z = sReg[GZ] * kGyroScale;

            // Orientation from on-board quaternion (w, x, y, z)
            imu_msg.orientation.w = sReg[q0] * kQuatScale;
            imu_msg.orientation.x = sReg[q1] * kQuatScale;
            imu_msg.orientation.y = sReg[q2] * kQuatScale;
            imu_msg.orientation.z = sReg[q3] * kQuatScale;

            imu_msg.orientation_covariance[0] = 0.01;
            imu_msg.orientation_covariance[4] = 0.01;
            imu_msg.orientation_covariance[8] = 0.05;
            imu_msg.angular_velocity_covariance[0] = 0.02;
            imu_msg.angular_velocity_covariance[4] = 0.02;
            imu_msg.angular_velocity_covariance[8] = 0.02;
            imu_msg.linear_acceleration_covariance[0] = 0.1;
            imu_msg.linear_acceleration_covariance[4] = 0.1;
            imu_msg.linear_acceleration_covariance[8] = 0.1;

            imu_pub->publish(imu_msg);
            s_cDataUpdate &= ~(ACC_UPDATE | GYRO_UPDATE | QUAT_UPDATE | ANGLE_UPDATE);
        }

        // ---- Magnetic field ------------------------------------------------
        if (s_cDataUpdate & MAG_UPDATE)
        {
            sensor_msgs::msg::MagneticField mag_msg;
            mag_msg.header.stamp = stamp;
            mag_msg.header.frame_id = imu_frame;
            mag_msg.magnetic_field.x = static_cast<double>(sReg[HX]);
            mag_msg.magnetic_field.y = static_cast<double>(sReg[HY]);
            mag_msg.magnetic_field.z = static_cast<double>(sReg[HZ]);
            mag_pub->publish(mag_msg);

            // chip temperature comes alongside the magnetic-field frame
            sensor_msgs::msg::Temperature temp_msg;
            temp_msg.header.stamp = stamp;
            temp_msg.header.frame_id = imu_frame;
            temp_msg.temperature = sReg[TEMP] / 100.0;
            temp_pub->publish(temp_msg);

            s_cDataUpdate &= ~MAG_UPDATE;
        }

        // ---- Barometric pressure / altitude --------------------------------
        if (s_cDataUpdate & PRESS_UPDATE)
        {
            sensor_msgs::msg::FluidPressure press_msg;
            press_msg.header.stamp = stamp;
            press_msg.header.frame_id = imu_frame;
            press_msg.fluid_pressure = static_cast<double>(Reg32(PressureL, PressureH)); // Pa
            press_pub->publish(press_msg);
            s_cDataUpdate &= ~PRESS_UPDATE;
        }

        // ---- GPS fix -------------------------------------------------------
        if (s_cDataUpdate & GPS_UPDATE)
        {
            const int32_t lon_raw = Reg32(LonL, LonH);
            const int32_t lat_raw = Reg32(LatL, LatH);

            // NMEA ddmm.mmmmm with the decimal point removed
            const double lon_deg = lon_raw / 10000000 + (std::abs(lon_raw) % 10000000) / 100000.0 / 60.0 * (lon_raw < 0 ? -1.0 : 1.0);
            const double lat_deg = lat_raw / 10000000 + (std::abs(lat_raw) % 10000000) / 100000.0 / 60.0 * (lat_raw < 0 ? -1.0 : 1.0);

            sensor_msgs::msg::NavSatFix fix_msg;
            fix_msg.header.stamp = stamp;
            fix_msg.header.frame_id = gps_frame;
            fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                                     sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
            fix_msg.status.status = (sReg[SVNUM] > 0)
                                        ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                        : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            fix_msg.latitude = lat_deg;
            fix_msg.longitude = lon_deg;
            fix_msg.altitude = sReg[GPSHeight] / 10.0; // m

            const double hdop = sReg[HDOP] / 100.0;
            const double sigma = (hdop > 0.0 ? hdop : 1.0) * 2.5; // rough metres
            fix_msg.position_covariance[0] = sigma * sigma;
            fix_msg.position_covariance[4] = sigma * sigma;
            fix_msg.position_covariance[8] = (sigma * 2.0) * (sigma * 2.0);
            fix_msg.position_covariance_type =
                sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

            fix_pub->publish(fix_msg);
            s_cDataUpdate &= ~GPS_UPDATE;
        }

        // ---- GPS velocity (ground speed + heading) -------------------------
        if (s_cDataUpdate & VEL_UPDATE)
        {
            const double speed_ms = Reg32(GPSVL, GPSVH) / 1000.0 * kKmhToMs; // km/h -> m/s
            const double heading_rad = sReg[GPSYAW] / 100.0 * M_PI / 180.0;  // deg -> rad

            geometry_msgs::msg::TwistStamped vel_msg;
            vel_msg.header.stamp = stamp;
            vel_msg.header.frame_id = gps_frame;
            // ENU: x = east, y = north. Heading measured clockwise from north.
            vel_msg.twist.linear.x = speed_ms * std::sin(heading_rad);
            vel_msg.twist.linear.y = speed_ms * std::cos(heading_rad);
            vel_pub->publish(vel_msg);
            s_cDataUpdate &= ~VEL_UPDATE;
        }

        // ---- GNSS UTC time -------------------------------------------------
        if (s_cDataUpdate & TIME_UPDATE)
        {
            sensor_msgs::msg::TimeReference time_msg;
            time_msg.header.stamp = stamp;
            time_msg.header.frame_id = gps_frame;
            time_msg.source = time_ref_source;

            std::tm tm = {};
            tm.tm_year = (sReg[YYMM] & 0xFF) + 100;       // years since 1900 (20xx)
            tm.tm_mon = ((sReg[YYMM] >> 8) & 0xFF) - 1;   // 0-based month
            tm.tm_mday = (sReg[DDHH] & 0xFF);
            tm.tm_hour = ((sReg[DDHH] >> 8) & 0xFF);
            tm.tm_min = (sReg[MMSS] & 0xFF);
            tm.tm_sec = ((sReg[MMSS] >> 8) & 0xFF);
            const time_t utc = timegm(&tm);
            const uint16_t ms = static_cast<uint16_t>(sReg[MS]);
            time_msg.time_ref = rclcpp::Time(static_cast<int64_t>(utc) * 1000000000LL +
                                             static_cast<int64_t>(ms) * 1000000LL);
            time_pub->publish(time_msg);
            s_cDataUpdate &= ~TIME_UPDATE;
        }

        s_cDataUpdate &= ~(DOP_UPDATE | READ_UPDATE);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    serial_close(s_fd);
    rclcpp::shutdown();
    return 0;
}
