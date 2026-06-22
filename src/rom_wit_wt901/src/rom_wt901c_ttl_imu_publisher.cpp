//#define ROM_DEBUG

#include <chrono>
#include <functional>
#include <memory>
#include <cstdint>
#include <unistd.h>
#include <iostream>
#include <cmath>
#ifdef ROM_DEBUG
#include <cstdio>
#endif

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
extern "C"
{
#include "serial.h"
#include "REG.h"
#include "wit_c_sdk.h"
}
#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80

#ifdef ROM_DEBUG
// ANSI color codes for grouped IMU debug output on the terminal.
// Only compiled in when ROM_DEBUG is defined; otherwise behavior is unchanged.
#define ROM_C_RESET  "\033[0m"
#define ROM_C_HEADER "\033[1;37m" // bright white
#define ROM_C_ANGLE  "\033[1;36m" // cyan   - angle / yaw
#define ROM_C_ORIENT "\033[1;32m" // green  - orientation quaternion
#define ROM_C_GYRO   "\033[1;33m" // yellow - angular velocity
#endif

static int fd, s_iCurBaud = 9600;
static volatile char s_cDataUpdate = 0;

float fAcc[3], fGyro[3], fAngle[3];
unsigned char cBuff[1];
const int c_uiBaud[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

using namespace std::chrono_literals;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> publisher_;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pub_;
std::shared_ptr<rclcpp::TimerBase> timer_;

static void AutoScanSensor(const std::string &dev);
static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

// Bridge WIT SDK write/delay to our serial and sleep functions
static void WitSerialWriteCb(uint8_t *data, uint32_t len)
{
    if (fd >= 0)
    {
        serial_write_data(fd, data, static_cast<int>(len));
    }
}

static void WitDelayMsCb(uint16_t ms)
{
    Delayms(ms);
}

int main(int argc, char *argv[])
{
    // Initialize ROS 2 first to use logging and node features safely
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("wit_imu_publisher");

    // parameter: invert yaw sign (useful if sensor mounted reversed)
    node->declare_parameter<bool>("invert_yaw", false);
    bool invert_yaw = node->get_parameter("invert_yaw").as_bool();

    // Open serial device
    char deviceName[256] = "/dev/ttyS0"; // Adjust to your device (e.g., /dev/ttyS0)
    if ((fd = serial_open(deviceName, 9600)) < 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open %s", deviceName);
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Opened %s successfully.", deviceName);

    // Initialize WIT-Motion sensor and register callbacks
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(WitSerialWriteCb);
    WitDelayMsRegister(WitDelayMsCb);
    int cb_status = WitRegisterCallBack(SensorDataUpdate);
    RCLCPP_INFO(node->get_logger(), "WIT callback register status: %d", cb_status);

    // Configure sensor output (angles at 50 Hz). Best-effort; depends on device support
    WitSetContent(RSW_ANGLE);
    WitSetOutputRate(RRATE_50HZ);

    // Try to auto-detect baudrate/content if needed
    AutoScanSensor(deviceName);

    // Publishers
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu/out", 10); // imu/wit/out

    rclcpp::Rate rate(20.0); // 50 Hz main loop

    double last_yaw = 0.0;
    rclcpp::Time last_time = node->get_clock()->now();
    double initial_yaw = 0.0;
    bool initial_yaw_set = false;

    while (rclcpp::ok())
    {
        // Pump serial bytes into the WIT SDK parser
        while (serial_read_data(fd, cBuff, 1) > 0)
        {
            WitSerialDataIn(cBuff[0]);
        }

        if (s_cDataUpdate & ANGLE_UPDATE)
        {
            // Convert raw yaw to radians; scale = pi/32768
            double yaw = sReg[Yaw] * 0.00009587379924285257; // radians
            if (invert_yaw) {
                yaw = -yaw;
            }

            // Reset yaw to zero on first reading (same behavior as wheel odometry)
            if (!initial_yaw_set) {
                initial_yaw = yaw;
                initial_yaw_set = true;
                RCLCPP_INFO(node->get_logger(), "IMU initial yaw offset: %.3f rad (%.1f deg)", 
                            initial_yaw, initial_yaw * 180.0 / M_PI);
            }
            yaw -= initial_yaw;

            // Normalize to [-pi, pi]
            while (yaw > M_PI) yaw -= 2.0 * M_PI;
            while (yaw < -M_PI) yaw += 2.0 * M_PI;

            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = node->get_clock()->now();
            imu_msg.header.frame_id = "imu_link"; // wit_imu_link

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            imu_msg.orientation.x = q.x();
            imu_msg.orientation.y = q.y();
            imu_msg.orientation.z = q.z();
            imu_msg.orientation.w = q.w();

            // Estimate angular velocity z from yaw derivative (fallback if gyro not available)
            rclcpp::Time now = node->get_clock()->now();
            double dt = (now - last_time).seconds();
            double ang_vel_z = 0.0;
            if (dt > 0.0001) {
                // normalize shortest angle difference
                double d = yaw - last_yaw;
                // wrap to [-pi, pi]
                while (d > M_PI) d -= 2.0 * M_PI;
                while (d < -M_PI) d += 2.0 * M_PI;
                ang_vel_z = d / dt;
            }
            last_yaw = yaw;
            last_time = now;

            imu_msg.angular_velocity.x = 0.0;
            imu_msg.angular_velocity.y = 0.0;
            imu_msg.angular_velocity.z = ang_vel_z;

            // Set reasonable covariances so EKF will accept the measurement
            // small values indicate trust in measurement
            imu_msg.orientation_covariance[0] = 0.01;
            imu_msg.orientation_covariance[4] = 0.01;
            imu_msg.orientation_covariance[8] = 0.05;

            imu_msg.angular_velocity_covariance[0] = 0.02;
            imu_msg.angular_velocity_covariance[4] = 0.02;
            imu_msg.angular_velocity_covariance[8] = 0.02;

            imu_msg.linear_acceleration_covariance[0] = 0.1;
            imu_msg.linear_acceleration_covariance[4] = 0.1;
            imu_msg.linear_acceleration_covariance[8] = 0.1;

            imu_pub_->publish(imu_msg);

#ifdef ROM_DEBUG
            // Grouped, color-coded IMU data for easy reading on the terminal.
            std::printf(
                ROM_C_HEADER "==== WT901C IMU ====" ROM_C_RESET "\n"
                ROM_C_ANGLE  "  Angle | yaw : %8.3f rad  (%7.2f deg)" ROM_C_RESET "\n"
                ROM_C_ORIENT "  Orient| x=%6.3f y=%6.3f z=%6.3f w=%6.3f" ROM_C_RESET "\n"
                ROM_C_GYRO   "  Gyro  | wz  : %8.3f rad/s" ROM_C_RESET "\n\n",
                yaw, yaw * 180.0 / M_PI,
                imu_msg.orientation.x, imu_msg.orientation.y,
                imu_msg.orientation.z, imu_msg.orientation.w,
                ang_vel_z);
            std::fflush(stdout);
#endif

            // Clear the ANGLE_UPDATE flag
            s_cDataUpdate &= ~ANGLE_UPDATE;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    serial_close(fd);
    rclcpp::shutdown();
    return 0;
}

static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum)
{
    for (uint32_t i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
        case AZ:
            s_cDataUpdate |= ACC_UPDATE;
            break;
        case GZ:
            s_cDataUpdate |= GYRO_UPDATE;
            break;
        case HZ:
            s_cDataUpdate |= MAG_UPDATE;
            break;
        case Yaw:
            s_cDataUpdate |= ANGLE_UPDATE;
            break;
        default:
            s_cDataUpdate |= READ_UPDATE;
            break;
        }
        uiReg++;
    }
}

static void Delayms(uint16_t ucMs)
{
    usleep(ucMs * 1000);
}

static void AutoScanSensor(const std::string &dev)
{
    unsigned char cBuff[1];

    char deviceName[256];
    strncpy(deviceName, dev.c_str(), sizeof(deviceName) - 1);
    deviceName[sizeof(deviceName) - 1] = '\0';

    for (size_t i = 1; i < 10; i++)
    {
        serial_close(fd);
        s_iCurBaud = c_uiBaud[i];
    fd = serial_open(deviceName, c_uiBaud[i]);

        int iRetry = 2;
        do
        {
            s_cDataUpdate = 0;
            // Request some registers; requires serial write registration
            WitReadReg(AX, 3);
            Delayms(200);
            while (serial_read_data(fd, cBuff, 1))
            {
                WitSerialDataIn(cBuff[0]);
            }
            if (s_cDataUpdate != 0)
            {
                std::cout << c_uiBaud[i] << " baud sensor found.\n"
                          << std::endl;
                return;
            }
            iRetry--;
        } while (iRetry);
    }

    std::cerr << "Sensor not found. Please check your connection." << std::endl;
    // system("exit");
}