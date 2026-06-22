// IMU Visualizer - a small Qt5 GUI application to visualize the orientation
// of up to 3 IMU sensors. Each panel lets the user pick a sensor_msgs/Imu
// topic at runtime from a dropdown and shows the orientation on a compass
// circle together with the roll / pitch / yaw values in degrees.

#include <QApplication>
#include <QComboBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr int kNumPanels = 3;

// Convert a quaternion into roll / pitch / yaw angles (radians).
void quaternionToEuler(double x, double y, double z, double w,
                       double & roll, double & pitch, double & yaw)
{
  // roll (rotation around x-axis)
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (rotation around y-axis)
  const double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  // yaw (rotation around z-axis)
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

double radToDeg(double r) { return r * 180.0 / M_PI; }
}  // namespace

// ---------------------------------------------------------------------------
// CompassWidget: draws a circle with a needle pointing at the current yaw and
// prints the yaw value (degrees) in the middle.
// ---------------------------------------------------------------------------
class CompassWidget : public QWidget
{
public:
  explicit CompassWidget(QWidget * parent = nullptr)
  : QWidget(parent)
  {
    setMinimumSize(220, 220);
  }

  void setOrientation(double roll_deg, double pitch_deg, double yaw_deg, bool has_data)
  {
    roll_deg_ = roll_deg;
    pitch_deg_ = pitch_deg;
    yaw_deg_ = yaw_deg;
    has_data_ = has_data;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    const int side = std::min(width(), height());
    const QPointF center(width() / 2.0, height() / 2.0);
    const double radius = side / 2.0 - 12.0;

    // Outer circle.
    p.setPen(QPen(QColor(70, 70, 70), 2));
    p.setBrush(QColor(245, 245, 245));
    p.drawEllipse(center, radius, radius);

    // Degree ticks every 30 degrees + cardinal labels.
    p.setPen(QPen(QColor(120, 120, 120), 1));
    QFont tick_font = p.font();
    tick_font.setPointSize(8);
    p.setFont(tick_font);
    for (int deg = 0; deg < 360; deg += 30) {
      // ROS yaw is counterclockwise-positive, so positive angles go to the
      // left on screen. 0 deg points up.
      const double a = (-deg - 90) * M_PI / 180.0;
      const QPointF outer(center.x() + std::cos(a) * radius,
                          center.y() + std::sin(a) * radius);
      const QPointF inner(center.x() + std::cos(a) * (radius - 10),
                          center.y() + std::sin(a) * (radius - 10));
      p.drawLine(inner, outer);

      const QPointF label_pt(center.x() + std::cos(a) * (radius - 24),
                             center.y() + std::sin(a) * (radius - 24));
      const int label_deg = (deg > 180) ? deg - 360 : deg;
      p.drawText(QRectF(label_pt.x() - 16, label_pt.y() - 8, 32, 16),
                 Qt::AlignCenter, QString::number(label_deg));
    }

    if (has_data_) {
      // Needle pointing at yaw (0 deg = up, counterclockwise positive).
      const double a = (-yaw_deg_ - 90) * M_PI / 180.0;
      const QPointF tip(center.x() + std::cos(a) * (radius - 14),
                        center.y() + std::sin(a) * (radius - 14));
      const QPointF tail(center.x() - std::cos(a) * (radius * 0.35),
                         center.y() - std::sin(a) * (radius * 0.35));
      p.setPen(QPen(QColor(200, 40, 40), 4, Qt::SolidLine, Qt::RoundCap));
      p.drawLine(tail, tip);

      // Center hub.
      p.setBrush(QColor(40, 40, 40));
      p.setPen(Qt::NoPen);
      p.drawEllipse(center, 5, 5);

      // Yaw value in the middle bottom.
      p.setPen(QColor(20, 20, 20));
      QFont val_font = p.font();
      val_font.setPointSize(13);
      val_font.setBold(true);
      p.setFont(val_font);
      p.drawText(QRectF(center.x() - radius, center.y() + radius * 0.45,
                        radius * 2, 24),
                 Qt::AlignCenter,
                 QString("Yaw %1\u00B0").arg(yaw_deg_, 0, 'f', 1));
    } else {
      p.setPen(QColor(160, 160, 160));
      QFont val_font = p.font();
      val_font.setPointSize(11);
      p.setFont(val_font);
      p.drawText(rect(), Qt::AlignCenter, "No data");
    }
  }

private:
  double roll_deg_ = 0.0;
  double pitch_deg_ = 0.0;
  double yaw_deg_ = 0.0;
  bool has_data_ = false;
};

// ---------------------------------------------------------------------------
// ImuPanel: one IMU view (topic selector + compass + numeric readout).
// ---------------------------------------------------------------------------
class ImuPanel : public QGroupBox
{
public:
  ImuPanel(int index, rclcpp::Node::SharedPtr node, QWidget * parent = nullptr)
  : QGroupBox(QString("IMU %1").arg(index + 1), parent), node_(std::move(node))
  {
    auto * layout = new QVBoxLayout(this);

    // Topic selection row.
    auto * topic_row = new QHBoxLayout();
    topic_combo_ = new QComboBox(this);
    topic_combo_->setEditable(true);
    topic_combo_->setMinimumWidth(120);
    topic_combo_->setToolTip("Select a sensor_msgs/Imu topic");
    topic_combo_->addItem("/example1");
    topic_combo_->addItem("/example2");
    topic_combo_->addItem("/example3");
    topic_combo_->setCurrentIndex(index % topic_combo_->count());
    subscribe_btn_ = new QPushButton("Subscribe", this);
    subscribe_btn_->setToolTip("Subscribe to the selected topic");
    topic_row->addWidget(new QLabel("Topic:", this));
    topic_row->addWidget(topic_combo_, 1);
    topic_row->addWidget(subscribe_btn_);
    layout->addLayout(topic_row);

    // Compass.
    compass_ = new CompassWidget(this);
    layout->addWidget(compass_, 1);

    // Numeric readout.
    rpy_label_ = new QLabel("Roll: --\u00B0   Pitch: --\u00B0   Yaw: --\u00B0", this);
    rpy_label_->setAlignment(Qt::AlignCenter);
    status_label_ = new QLabel("Not subscribed", this);
    status_label_->setAlignment(Qt::AlignCenter);
    status_label_->setStyleSheet("color: gray;");
    layout->addWidget(rpy_label_);
    layout->addWidget(status_label_);

    connect(subscribe_btn_, &QPushButton::clicked, this,
            [this]() { subscribeTo(topic_combo_->currentText()); });
  }

  // Called periodically by the GUI timer to refresh the displayed values.
  void updateDisplay()
  {
    double roll, pitch, yaw;
    bool has_data;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      roll = roll_deg_;
      pitch = pitch_deg_;
      yaw = yaw_deg_;
      has_data = has_data_;
    }

    compass_->setOrientation(roll, pitch, yaw, has_data);
    if (has_data) {
      rpy_label_->setText(QString("Roll: %1\u00B0   Pitch: %2\u00B0   Yaw: %3\u00B0")
                            .arg(roll, 0, 'f', 1)
                            .arg(pitch, 0, 'f', 1)
                            .arg(yaw, 0, 'f', 1));
    }
  }

private:
  void subscribeTo(const QString & topic)
  {
    subscription_.reset();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      has_data_ = false;
    }

    const std::string topic_name = topic.trimmed().toStdString();
    if (topic_name.empty()) {
      status_label_->setText("Not subscribed");
      status_label_->setStyleSheet("color: gray;");
      return;
    }

    subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      topic_name, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        double roll, pitch, yaw;
        quaternionToEuler(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w,
                          roll, pitch, yaw);
        std::lock_guard<std::mutex> lock(data_mutex_);
        roll_deg_ = radToDeg(roll);
        pitch_deg_ = radToDeg(pitch);
        yaw_deg_ = radToDeg(yaw);
        has_data_ = true;
      });

    status_label_->setText(QString("Subscribed: %1").arg(topic));
    status_label_->setStyleSheet("color: green;");
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;

  QComboBox * topic_combo_ = nullptr;
  QPushButton * subscribe_btn_ = nullptr;
  CompassWidget * compass_ = nullptr;
  QLabel * rpy_label_ = nullptr;
  QLabel * status_label_ = nullptr;

  std::mutex data_mutex_;
  double roll_deg_ = 0.0;
  double pitch_deg_ = 0.0;
  double yaw_deg_ = 0.0;
  bool has_data_ = false;
};

// ---------------------------------------------------------------------------
// Main window holding the three IMU panels.
// ---------------------------------------------------------------------------
class MainWindow : public QWidget
{
public:
  explicit MainWindow(rclcpp::Node::SharedPtr node)
  {
    setWindowTitle("ROM IMU Visualizer");

    auto * root = new QVBoxLayout(this);

    auto * panel_row = new QHBoxLayout();
    for (int i = 0; i < kNumPanels; ++i) {
      auto * panel = new ImuPanel(i, node, this);
      panels_.push_back(panel);
      panel_row->addWidget(panel);
    }
    root->addLayout(panel_row, 1);

    // GUI-thread timer that pulls the latest data from each panel.
    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
      for (auto * panel : panels_) {
        panel->updateDisplay();
      }
    });
    timer->start(33);  // ~30 Hz
  }

private:
  std::vector<ImuPanel *> panels_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("imu_visualizer");

  // Spin ROS in a background thread so the Qt event loop stays responsive.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::atomic<bool> running{true};
  std::thread ros_thread([&executor, &running]() {
    while (running && rclcpp::ok()) {
      executor.spin_some(std::chrono::milliseconds(50));
    }
  });

  MainWindow window(node);
  window.resize(820, 460);
  window.show();

  const int ret = app.exec();

  running = false;
  if (ros_thread.joinable()) {
    ros_thread.join();
  }
  rclcpp::shutdown();
  return ret;
}
