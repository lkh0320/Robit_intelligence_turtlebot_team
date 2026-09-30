#include "turtle_gui/main_window.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <utility>

#include <QApplication>
#include <QCheckBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QScreen>
#include <QPushButton>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>


namespace
{
constexpr int kStaleMs = 1000;   // 이 시간 이상 수신이 없으면 빨간 램프

QString onOff(bool v)
{
  return v ? "<b style='color:#2e7d32'>검출</b>" : "<span style='color:gray'>없음</span>";
}

QString lightName(uint8_t s)
{
  using M = interfaces::msg::TrafficLight;
  switch (s) {
    case M::RED: return "<b style='color:#d32f2f'>RED</b>";
    case M::YELLOW: return "<b style='color:#f9a825'>YELLOW</b>";
    case M::GREEN: return "<b style='color:#2e7d32'>GREEN</b>";
    default: return "UNKNOWN";
  }
}

QString sideName(uint8_t s)
{
  // Sign / ParkingSpot 은 NONE=0, LEFT=1, RIGHT=2 로 같다
  switch (s) {
    case 1: return "LEFT";
    case 2: return "RIGHT";
    default: return "NONE";
  }
}

QString barrierName(uint8_t s)
{
  using M = interfaces::msg::Barrier;
  switch (s) {
    case M::CLOSED: return "<b style='color:#d32f2f'>CLOSED</b>";
    case M::OPEN: return "<b style='color:#2e7d32'>OPEN</b>";
    default: return "UNKNOWN";
  }
}

QString modeName(uint8_t m)
{
  using M = interfaces::msg::ControlMode;
  switch (m) {
    case M::STOP: return "STOP";
    case M::LANE: return "LANE";
    case M::AVOID: return "AVOID";
    case M::PARKING: return "PARKING";
    case M::MANUAL: return "MANUAL";
    default: return QString("? (%1)").arg(m);
  }
}
}  // namespace

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent)
: QMainWindow(parent), node_(std::move(node))
{
  wheel_separation_ = node_->declare_parameter("wheel_separation", 0.160);
  psd_max_range_ = node_->declare_parameter("psd_max_range", 0.80);

  cams_[0].key = "cam_raw";
  cams_[0].topic = QString::fromStdString(
    node_->declare_parameter("image_topic", std::string("image_raw/compressed")));
  cams_[1].key = "cam_bev";
  cams_[1].topic = QString::fromStdString(
    node_->declare_parameter("bev_image_topic", std::string("image_bev/compressed")));
  cams_[2].key = "cam_lane";
  cams_[2].topic = QString::fromStdString(
    node_->declare_parameter("lane_image_topic", std::string("vision/lane_debug/compressed")));
  cams_[3].key = "cam_object";
  cams_[3].topic = QString::fromStdString(
    node_->declare_parameter("object_image_topic", std::string("vision/object_debug/compressed")));

  setWindowTitle("TurtleBot Test GUI");
  // 기본 1600x900, 화면이 더 작으면 화면에 맞춤
  const QSize avail = QGuiApplication::primaryScreen()->availableGeometry().size();
  resize(QSize(1600, 900).boundedTo(avail));

  // 왼쪽: 카메라 4개 (2x2) + 로그 / 오른쪽: 상태 + 조종
  //   원본         | Bird's Eye View
  //   선·벡터 검출 | 객체 인식
  // (카메라 밝기/노출은 GUI 가 아니라 Jetson 에서 v4l2-ctl 로 설정: vision_bringup/camera/README.md)
  auto * left = new QWidget;
  auto * left_layout = new QVBoxLayout(left);
  auto * cam_grid = new QGridLayout;
  cam_grid->addWidget(buildCameraView(cams_[0], "원본"), 0, 0);
  cam_grid->addWidget(buildCameraView(cams_[1], "Bird's Eye View"), 0, 1);
  cam_grid->addWidget(buildCameraView(cams_[2], "선 · 벡터 검출"), 1, 0);
  cam_grid->addWidget(buildCameraView(cams_[3], "객체 인식"), 1, 1);
  for (int i = 0; i < 2; ++i) {
    cam_grid->setRowStretch(i, 1);
    cam_grid->setColumnStretch(i, 1);
  }
  log_view_ = new QPlainTextEdit;
  log_view_->setReadOnly(true);
  log_view_->setMaximumBlockCount(500);
  log_view_->setMaximumHeight(120);
  left_layout->addLayout(cam_grid, 1);
  left_layout->addWidget(log_view_);

  auto * right = new QWidget;
  auto * right_layout = new QVBoxLayout(right);
  right_layout->addWidget(buildStatusPanel(), 1);
  right_layout->addWidget(buildControlPanel());

  auto * splitter = new QSplitter;
  splitter->addWidget(left);
  splitter->addWidget(right);
  splitter->setStretchFactor(0, 5);
  splitter->setStretchFactor(1, 2);
  setCentralWidget(splitter);

  setupRos();

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, [this] {
    if (rclcpp::ok()) {
      rclcpp::spin_some(node_);
    } else {
      close();
    }
  });
  spin_timer_->start(5);

  cmd_timer_ = new QTimer(this);
  connect(cmd_timer_, &QTimer::timeout, this, &MainWindow::sendManualCmd);
  cmd_timer_->start(100);   // 10 Hz

  lamp_timer_ = new QTimer(this);
  connect(lamp_timer_, &QTimer::timeout, this, &MainWindow::refreshLamps);
  lamp_timer_->start(1000);

  // 창이 비활성화되면 눌린 키를 모두 뗀 것으로 처리 (키가 눌린 채로 남는 것 방지)
  connect(qApp, &QGuiApplication::applicationStateChanged, this, [this](Qt::ApplicationState s) {
    if (s != Qt::ApplicationActive) {
      pressed_keys_.clear();
    }
  });
  qApp->installEventFilter(this);

  log("turtle_gui 시작");
}

QWidget * MainWindow::buildStatusPanel()
{
  auto * panel = new QWidget;
  auto * layout = new QVBoxLayout(panel);
  layout->setContentsMargins(0, 0, 0, 0);

  // 토픽 연결 상태
  auto * topic_box = new QGroupBox("토픽 수신 상태");
  auto * grid = new QGridLayout(topic_box);
  int row = 0;
  addTopicRow(grid, row++, "cam_raw", "카메라 원본");
  addTopicRow(grid, row++, "cam_bev", "BEV 화면");
  addTopicRow(grid, row++, "cam_lane", "선 검출 화면");
  addTopicRow(grid, row++, "cam_object", "객체 인식 화면");
  addTopicRow(grid, row++, "psd", "psd");
  addTopicRow(grid, row++, "dxl", "dxl_state");
  addTopicRow(grid, row++, "cmd", "cmd_vel");
  addTopicRow(grid, row++, "mode", "control_mode");
  row = 0;
  auto * grid2 = new QGridLayout;
  addTopicRow(grid2, row++, "lane", "lane_info");
  addTopicRow(grid2, row++, "light", "traffic_light");
  addTopicRow(grid2, row++, "stop", "stop_line");
  addTopicRow(grid2, row++, "sign", "sign");
  addTopicRow(grid2, row++, "barrier", "barrier");
  addTopicRow(grid2, row++, "parking", "parking_spot");
  grid->addLayout(grid2, 0, 2, row, 1);
  layout->addWidget(topic_box);

  // PSD
  auto * psd_box = new QGroupBox("PSD [m]");
  auto * psd_grid = new QGridLayout(psd_box);
  const char * names[3] = {"왼쪽", "앞", "오른쪽"};
  for (int i = 0; i < 3; ++i) {
    psd_bar_[i] = new QProgressBar;
    psd_bar_[i]->setRange(0, static_cast<int>(psd_max_range_ * 1000));
    psd_bar_[i]->setTextVisible(false);
    psd_text_[i] = new QLabel("-");
    psd_text_[i]->setMinimumWidth(60);
    psd_grid->addWidget(new QLabel(names[i]), i, 0);
    psd_grid->addWidget(psd_bar_[i], i, 1);
    psd_grid->addWidget(psd_text_[i], i, 2);
  }
  layout->addWidget(psd_box);

  // 모터
  auto * motor_box = new QGroupBox("모터");
  auto * motor_form = new QFormLayout(motor_box);
  dxl_label_ = new QLabel("-");
  cmd_label_ = new QLabel("-");
  mode_label_ = new QLabel("-");
  motor_form->addRow("DXL 실제", dxl_label_);
  motor_form->addRow("cmd_vel", cmd_label_);
  motor_form->addRow("제어 모드", mode_label_);
  layout->addWidget(motor_box);

  // 비전
  auto * vision_box = new QGroupBox("비전 인식");
  auto * vision_form = new QFormLayout(vision_box);
  lane_label_ = new QLabel("-");
  light_label_ = new QLabel("-");
  stop_label_ = new QLabel("-");
  sign_label_ = new QLabel("-");
  barrier_label_ = new QLabel("-");
  parking_label_ = new QLabel("-");
  vision_form->addRow("차선", lane_label_);
  vision_form->addRow("신호등", light_label_);
  vision_form->addRow("정지선", stop_label_);
  vision_form->addRow("표지판", sign_label_);
  vision_form->addRow("차단바", barrier_label_);
  vision_form->addRow("주차", parking_label_);
  layout->addWidget(vision_box);

  layout->addStretch();
  return panel;
}

void MainWindow::addTopicRow(QGridLayout * grid, int row, const QString & key, const QString & topic)
{
  auto * lamp = new QLabel("●  -");
  lamp->setStyleSheet("color:gray;");
  lamp->setMinimumWidth(80);
  grid->addWidget(new QLabel(topic), row, 0);
  grid->addWidget(lamp, row, 1);
  stats_[key].lamp = lamp;
}

QWidget * MainWindow::buildControlPanel()
{
  auto * box = new QGroupBox("조종 (송신)");
  auto * layout = new QVBoxLayout(box);

  // 모드 강제 변경
  auto * mode_row = new QHBoxLayout;
  mode_row->addWidget(new QLabel("control_mode:"));
  using M = interfaces::msg::ControlMode;
  for (uint8_t m : {M::STOP, M::LANE, M::AVOID, M::PARKING, M::MANUAL}) {
    auto * btn = new QPushButton(modeName(m));
    btn->setFocusPolicy(Qt::NoFocus);
    connect(btn, &QPushButton::clicked, this, [this, m] {sendControlMode(m);});
    mode_row->addWidget(btn);
  }
  layout->addLayout(mode_row);

  // 수동 주행
  manual_enable_ = new QCheckBox("수동 주행 (cmd_vel 발행)  — W/A/S/D, Space=정지");
  manual_enable_->setFocusPolicy(Qt::NoFocus);
  connect(manual_enable_, &QCheckBox::toggled, this, [this](bool on) {
    pressed_keys_.clear();
    log(on ? "수동 주행 ON: cmd_vel 발행 시작" : "수동 주행 OFF");
    if (!on) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      was_moving_ = false;
    }
  });
  layout->addWidget(manual_enable_);

  auto * speed_row = new QHBoxLayout;
  lin_speed_ = new QDoubleSpinBox;
  lin_speed_->setRange(0.0, 0.26);
  lin_speed_->setSingleStep(0.01);
  lin_speed_->setValue(0.10);
  lin_speed_->setSuffix(" m/s");
  ang_speed_ = new QDoubleSpinBox;
  ang_speed_->setRange(0.0, 3.0);
  ang_speed_->setSingleStep(0.1);
  ang_speed_->setValue(1.0);
  ang_speed_->setSuffix(" rad/s");
  speed_row->addWidget(new QLabel("선속도"));
  speed_row->addWidget(lin_speed_);
  speed_row->addWidget(new QLabel("각속도"));
  speed_row->addWidget(ang_speed_);
  layout->addLayout(speed_row);

  // 화면 버튼 (누르고 있는 동안 이동)
  auto * pad = new QGridLayout;
  auto make_key = [this, pad](const QString & text, int key, int r, int c) {
      auto * btn = new QPushButton(text);
      btn->setFocusPolicy(Qt::NoFocus);
      btn->setMinimumHeight(36);
      connect(btn, &QPushButton::pressed, this, [this, key] {pressed_keys_.insert(key);});
      connect(btn, &QPushButton::released, this, [this, key] {pressed_keys_.erase(key);});
      pad->addWidget(btn, r, c);
    };
  make_key("W ▲", Qt::Key_W, 0, 1);
  make_key("A ◀", Qt::Key_A, 1, 0);
  make_key("S ▼", Qt::Key_S, 1, 1);
  make_key("D ▶", Qt::Key_D, 1, 2);
  auto * stop_btn = new QPushButton("정지 (Space)");
  stop_btn->setFocusPolicy(Qt::NoFocus);
  stop_btn->setStyleSheet("background:#d32f2f; color:white; font-weight:bold;");
  stop_btn->setMinimumHeight(36);
  connect(stop_btn, &QPushButton::clicked, this, [this] {
    pressed_keys_.clear();
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    was_moving_ = false;
  });
  pad->addWidget(stop_btn, 0, 2);
  layout->addLayout(pad);

  return box;
}

void MainWindow::setupRos()
{
  const std::string cmd_topic = node_->declare_parameter("cmd_vel_topic", std::string("cmd_vel"));

  // 발행 측 QoS(best effort / reliable)에 상관없이 받도록 센서 QoS로 구독
  const auto qos = rclcpp::SensorDataQoS();

  // --- 카메라 ---
  for (auto & view : cams_) {
    subscribeCamera(view);
  }

  // --- stm ---
  psd_sub_ = node_->create_subscription<interfaces::msg::PsdArray>(
    "psd", qos, [this](interfaces::msg::PsdArray::ConstSharedPtr msg) {
      touch("psd");
      const float v[3] = {msg->left, msg->front, msg->right};
      for (int i = 0; i < 3; ++i) {
        psd_bar_[i]->setValue(std::clamp(static_cast<int>(v[i] * 1000), 0, psd_bar_[i]->maximum()));
        psd_text_[i]->setText(QString::number(v[i], 'f', 3));
      }
    });

  dxl_sub_ = node_->create_subscription<interfaces::msg::DxlState>(
    "dxl_state", qos, [this](interfaces::msg::DxlState::ConstSharedPtr msg) {
      touch("dxl");
      QString text = QString("L %1  R %2 m/s").arg(msg->left_velocity, 6, 'f', 3)
        .arg(msg->right_velocity, 6, 'f', 3);
      if (msg->left_error || msg->right_error) {
        text += QString("   <b style='color:#d32f2f'>ERR L=0x%1 R=0x%2</b>")
          .arg(msg->left_error, 2, 16, QChar('0')).arg(msg->right_error, 2, 16, QChar('0'));
      }
      dxl_label_->setText(text);
    });

  // 실제로 로봇에 들어가는 cmd_vel (GUI가 보낸 것 포함) + stm과 같은 식으로 바퀴 목표속도 계산
  cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    cmd_topic, 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
      touch("cmd");
      const double v = msg->linear.x, w = msg->angular.z;
      cmd_label_->setText(QString("v %1 m/s  w %2 rad/s  →  목표 L %3  R %4")
        .arg(v, 0, 'f', 2).arg(w, 0, 'f', 2)
        .arg(v - w * wheel_separation_ / 2.0, 0, 'f', 3)
        .arg(v + w * wheel_separation_ / 2.0, 0, 'f', 3));
    });

  mode_sub_ = node_->create_subscription<interfaces::msg::ControlMode>(
    "control_mode", 10, [this](interfaces::msg::ControlMode::ConstSharedPtr msg) {
      touch("mode");
      mode_label_->setText(QString("<b>%1</b>  %2").arg(modeName(msg->mode))
        .arg(QString::fromStdString(msg->mission_state)));
    });

  // --- vision ---
  lane_sub_ = node_->create_subscription<interfaces::msg::LaneInfo>(
    "lane_info", qos, [this](interfaces::msg::LaneInfo::ConstSharedPtr m) {
      touch("lane");
      lane_label_->setText(QString("%1  offset %2  angle %3 rad  (%4)").arg(onOff(m->detected))
        .arg(m->offset, 0, 'f', 2).arg(m->angle, 0, 'f', 2).arg(m->confidence, 0, 'f', 2));
    });
  light_sub_ = node_->create_subscription<interfaces::msg::TrafficLight>(
    "traffic_light", qos, [this](interfaces::msg::TrafficLight::ConstSharedPtr m) {
      touch("light");
      light_label_->setText(QString("%1  (%2)").arg(lightName(m->state))
        .arg(m->confidence, 0, 'f', 2));
    });
  stop_sub_ = node_->create_subscription<interfaces::msg::StopLine>(
    "stop_line", qos, [this](interfaces::msg::StopLine::ConstSharedPtr m) {
      touch("stop");
      stop_label_->setText(QString("%1  y %2  (%3)").arg(onOff(m->detected))
        .arg(m->y_ratio, 0, 'f', 2).arg(m->confidence, 0, 'f', 2));
    });
  sign_sub_ = node_->create_subscription<interfaces::msg::Sign>(
    "sign", qos, [this](interfaces::msg::Sign::ConstSharedPtr m) {
      touch("sign");
      sign_label_->setText(QString("<b>%1</b>  area %2  (%3)").arg(sideName(m->type))
        .arg(m->area_ratio, 0, 'f', 3).arg(m->confidence, 0, 'f', 2));
    });
  barrier_sub_ = node_->create_subscription<interfaces::msg::Barrier>(
    "barrier", qos, [this](interfaces::msg::Barrier::ConstSharedPtr m) {
      touch("barrier");
      barrier_label_->setText(QString("%1  %2  (%3)").arg(onOff(m->detected))
        .arg(barrierName(m->state)).arg(m->confidence, 0, 'f', 2));
    });
  parking_sub_ = node_->create_subscription<interfaces::msg::ParkingSpot>(
    "parking_spot", qos, [this](interfaces::msg::ParkingSpot::ConstSharedPtr m) {
      touch("parking");
      parking_label_->setText(QString("%1  빈칸 <b>%2</b>  offset %3  (%4)").arg(onOff(m->detected))
        .arg(sideName(m->empty_side)).arg(m->offset, 0, 'f', 2).arg(m->confidence, 0, 'f', 2));
    });

  // --- 송신 ---
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
  mode_pub_ = node_->create_publisher<interfaces::msg::ControlMode>("control_mode", 10);
}

void MainWindow::touch(const QString & key)
{
  auto & s = stats_[key];
  if (!s.last.isValid()) {
    log(QString("첫 수신: %1").arg(key));
  }
  s.last.start();
  ++s.count;
  if (s.lamp && !s.lamp->styleSheet().contains("green")) {
    s.lamp->setStyleSheet("color:green;");
  }
}

void MainWindow::refreshLamps()
{
  // 1초마다 호출 -> count 가 곧 Hz
  for (auto & [key, s] : stats_) {
    if (!s.lamp || !s.last.isValid()) {
      continue;
    }
    const bool stale = s.last.elapsed() > kStaleMs;
    s.lamp->setStyleSheet(stale ? "color:#d32f2f;" : "color:green;");
    s.lamp->setText(QString("●  %1 Hz").arg(s.count));
    s.count = 0;
  }
}

void MainWindow::sendManualCmd()
{
  if (!manual_enable_->isChecked()) {
    return;
  }
  double v = 0.0, w = 0.0;
  if (pressed_keys_.count(Qt::Key_W)) {v += lin_speed_->value();}
  if (pressed_keys_.count(Qt::Key_S)) {v -= lin_speed_->value();}
  if (pressed_keys_.count(Qt::Key_A)) {w += ang_speed_->value();}
  if (pressed_keys_.count(Qt::Key_D)) {w -= ang_speed_->value();}

  const bool moving = v != 0.0 || w != 0.0;
  // 움직이는 동안 10 Hz로 계속 발행, 멈추면 정지 명령 1번만 (다른 노드의 cmd_vel과 덜 싸우도록)
  if (moving || was_moving_) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = v;
    msg.angular.z = w;
    cmd_pub_->publish(msg);
  }
  was_moving_ = moving;
}

void MainWindow::sendControlMode(uint8_t mode)
{
  interfaces::msg::ControlMode msg;
  msg.header.stamp = node_->now();
  msg.mode = mode;
  msg.mission_state = "turtle_gui";
  mode_pub_->publish(msg);
  log(QString("control_mode 발행: %1").arg(modeName(mode)));
}

bool MainWindow::eventFilter(QObject * obj, QEvent * event)
{
  if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
    auto * ke = static_cast<QKeyEvent *>(event);
    const int key = ke->key();
    if (key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D) {
      if (!ke->isAutoRepeat()) {
        if (event->type() == QEvent::KeyPress) {
          pressed_keys_.insert(key);
        } else {
          pressed_keys_.erase(key);
        }
      }
      return true;
    }
    if (key == Qt::Key_Space) {
      if (event->type() == QEvent::KeyPress && !ke->isAutoRepeat()) {
        pressed_keys_.clear();
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        was_moving_ = false;
      }
      return true;
    }
  }
  return QMainWindow::eventFilter(obj, event);
}

void MainWindow::log(const QString & text)
{
  log_view_->appendPlainText(
    QDateTime::currentDateTime().toString("hh:mm:ss  ") + text);
}

QWidget * MainWindow::buildCameraView(CameraView & view, const QString & title)
{
  auto * box = new QGroupBox(title);
  auto * layout = new QVBoxLayout(box);
  layout->setContentsMargins(4, 4, 4, 4);
  view.image = new QLabel("수신 대기 중\n" + view.topic);
  view.image->setAlignment(Qt::AlignCenter);
  view.image->setMinimumSize(240, 180);
  view.image->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  view.image->setStyleSheet("background:#202020; color:#aaaaaa;");
  view.info = new QLabel("-");
  layout->addWidget(view.image, 1);
  layout->addWidget(view.info);
  view.fps_timer.start();
  return box;
}

void MainWindow::subscribeCamera(CameraView & view)
{
  const std::string topic = view.topic.toStdString();
  const auto qos = rclcpp::SensorDataQoS();
  CameraView * v = &view;   // cams_ 는 std::array 라 주소가 바뀌지 않음

  if (view.topic.endsWith("/compressed")) {
    view.sub = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic, qos, [this, v](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
        QImage img;
        if (img.loadFromData(msg->data.data(), static_cast<int>(msg->data.size()))) {
          onFrame(*v, img);
        } else {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "Failed to decode compressed image on %s (format=%s)",
            v->topic.toStdString().c_str(), msg->format.c_str());
        }
      });
  } else {
    view.sub = node_->create_subscription<sensor_msgs::msg::Image>(
      topic, qos, [this, v](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        const auto * data = msg->data.data();
        const int w = static_cast<int>(msg->width), h = static_cast<int>(msg->height);
        const int step = static_cast<int>(msg->step);
        QImage img;
        if (msg->encoding == "rgb8") {
          img = QImage(data, w, h, step, QImage::Format_RGB888).copy();
        } else if (msg->encoding == "bgr8") {
          img = QImage(data, w, h, step, QImage::Format_RGB888).rgbSwapped();
        } else if (msg->encoding == "mono8") {
          img = QImage(data, w, h, step, QImage::Format_Grayscale8).copy();
        } else if (msg->encoding == "rgba8") {
          img = QImage(data, w, h, step, QImage::Format_RGBA8888).copy();
        } else if (msg->encoding == "bgra8") {
          img = QImage(data, w, h, step, QImage::Format_ARGB32).copy();
        } else {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "Unsupported image encoding on %s: %s",
            v->topic.toStdString().c_str(), msg->encoding.c_str());
          return;
        }
        onFrame(*v, img);
      });
  }
  log(QString("카메라 구독: %1").arg(view.topic));
}

void MainWindow::onFrame(CameraView & view, const QImage & image)
{
  touch(view.key);
  ++view.frames;
  if (view.fps_timer.elapsed() >= 1000) {
    view.fps = view.frames * 1000.0 / view.fps_timer.restart();
    view.frames = 0;
  }
  view.info->setText(QString("%1 x %2   %3 fps   %4").arg(image.width()).arg(image.height())
    .arg(view.fps, 0, 'f', 1).arg(view.topic));
  view.image->setPixmap(QPixmap::fromImage(image).scaled(
      view.image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
