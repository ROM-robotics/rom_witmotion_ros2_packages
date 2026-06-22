# rom_wit_wt901

> **ROM Robotics** — WIT-Motion IMU Publisher (ROS 2 Humble)

WIT-Motion IMU sensor မှ yaw angle data ကို serial port မှတဆင့် ဖတ်ပြီး `sensor_msgs/msg/Imu` message အဖြစ် publish ထုတ်ပေးသော package ဖြစ်ပါတယ်။

---

## 🌐 Namespace Support

Topic `imu/out` သည် **relative path** (leading `/` မပါ) ဖြစ်သောကြောင့် `PushRosNamespace` ဖြင့် namespace push လုပ်ပါက topics အားလုံး `/<namespace>/...` အောက်သို့ ရောက်ပါမည်။

| Environment Variable | Default | Description |
|---|---|---|
| `ROM_ROBOT_NAMESPACE` | `default_robot1` | Launch file မှ `PushRosNamespace` ဖြင့် push လုပ်ပါသည် |

```bash
# Example: namespace = bobo01
export ROM_ROBOT_NAMESPACE=bobo01
# Topic: /bobo01/imu/out
```

> ⚠️ ဤ node ကို `hardware.launch.py` (`bobo_controller` package) မှ namespace-aware GroupAction ထဲတွင် launch ပါသည်။

---

## 📁 Package Structure

```
rom_wit_wt901/
├── CMakeLists.txt
├── package.xml
├── README.md
└── src/
    ├── imupublisher_updated.cpp   # Active — 2-camera merge + EKF-ready
    ├── imupublisher.cpp           # Legacy (not built)
    ├── REG.h                      # WIT sensor register definitions
    ├── serial.c                   # Serial port implementation
    ├── serial.h                   # Serial port header
    ├── wit_c_sdk.c                # WIT-Motion C SDK
    └── wit_c_sdk.h                # WIT-Motion SDK header
```

---

## 🚀 Node: `imu` (from `imupublisher_updated.cpp`)

### Published Topics

| Topic | Message Type | Description |
|---|---|---|
| `imu/out` | `sensor_msgs/msg/Imu` | Orientation (quaternion), angular velocity, covariance (relative topic) |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `invert_yaw` | `bool` | `false` | Yaw sign ပြောင်းရန် (sensor ပြောင်းပြန်တပ်ထားရင်) |

### Serial Configuration

| Setting | Value |
|---|---|
| Device | `/dev/ttyS0` |
| Baud rate | `9600` (auto-scan 2400–921600) |
| Protocol | WIT_PROTOCOL_NORMAL |
| Output | Angle data at 50 Hz |

---

## 🔀 Data Flow

```mermaid
graph LR
    IMU["🔄 WIT-Motion\nIMU Sensor"]
    SERIAL["/dev/ttyS0\n(Serial)"]
    NODE["imu node\n(wit_imu_publisher)"]
    TOPIC(["imu/out\n(sensor_msgs/Imu)\nnamespace-aware"])
    EKF["🗺️ robot_localization\n(EKF)"]

    IMU -- "hardware" --> SERIAL
    SERIAL -- "serial_read_data()" --> NODE
    NODE -- "publish" --> TOPIC
    TOPIC --> EKF

    style IMU fill:#FFB74D,stroke:#F57C00,color:#000
    style SERIAL fill:#CFD8DC,stroke:#37474F,color:#000
    style NODE fill:#CE93D8,stroke:#6A1B9A,color:#000
    style TOPIC fill:#FFF176,stroke:#F9A825,color:#000
    style EKF fill:#A5D6A7,stroke:#2E7D32,color:#000
```

---

## 📐 IMU Output Details

### Orientation (Quaternion from Yaw)

$$
q = \text{setRPY}(0, 0, \theta_{\text{yaw}})
$$

$$
\theta_{\text{yaw}} = \text{sReg}[\text{Yaw}] \times \frac{\pi}{32768}
$$

Yaw ကို `[-\pi, \pi]` range သို့ normalize လုပ်ပြီး initial offset ကို zero reset လုပ်ပါတယ်။

### Angular Velocity (Estimated)

$$
\omega_z = \frac{\Delta\theta}{\Delta t}
$$

Gyro data မရရင် yaw derivative ဖြင့် angular velocity ကို estimate လုပ်ပါတယ်။

### Covariance

| Field | Diagonal Values | Meaning |
|---|---|---|
| Orientation | `[0.01, 0.01, 0.05]` | Yaw uncertainty ပို |
| Angular Velocity | `[0.02, 0.02, 0.02]` | Moderate trust |
| Linear Acceleration | `[0.1, 0.1, 0.1]` | Lower trust |

---

## 🏗️ Build & Run

### Build

```bash
cd ~/rom_drivers_ws
colcon build --packages-select rom_wit_wt901
source install/setup.bash
```

### Run

```bash
# Recommended: launch via hardware.launch.py (namespace-aware)
export ROM_ROBOT_NAMESPACE=bobo01
ros2 launch bobo_controller hardware.launch.py use_imu:=true

# Standalone (no namespace)
ros2 run rom_wit_wt901 imu
```

---

*ROM Robotics © 2026*
