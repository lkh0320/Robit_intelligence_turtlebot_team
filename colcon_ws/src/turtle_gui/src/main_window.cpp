// MainWindow 구현: 화면 구성, ROS 구독/발행, 키보드 수동 주행
#include "turtle_gui/main_window.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
#include <QSignalBlocker>
#include <QSlider>
#include <QPushButton>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>


namespace
{
constexpr int kStaleMs = 1000;   // 이 시간 이상 수신이 없으면 빨간 램프

// 아래 함수들은 메시지 값을 라벨에 보여줄 글자(HTML 색 포함)로 바꾼다
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
  // 파라미터 (ros2 run turtle_gui turtle_gui --ros-args -p image_topic:=... 로 바꿀 수 있다)
  wheel_separation_ = node_->declare_parameter("wheel_separation", 0.160);
  psd_max_range_ = node_->declare_parameter("psd_max_range", 0.80);
  // 수동 주행 속도: 시작값과 슬라이더 상한 (GUI 에서 바로 바꿀 수 있고, 여기서는 처음 값만 정한다)
  lin_speed_init_ = node_->declare_parameter("linear_speed", 0.10);
  ang_speed_init_ = node_->declare_parameter("angular_speed", 1.0);
  lin_speed_max_ = node_->declare_parameter("max_linear_speed", 0.26);   // stm_bridge max_wheel_speed 와 같게
  ang_speed_max_ = node_->declare_parameter("max_angular_speed", 3.0);

  // 카메라 화면 4개가 볼 토픽
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
  // 아래쪽 로그창 (최대 500줄, 오래된 줄부터 지워짐)
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

  // 좌우 경계를 마우스로 끌어 크기 조절 가능. 처음 비율은 5 : 2
  auto * splitter = new QSplitter;
  splitter->addWidget(left);
  splitter->addWidget(right);
  splitter->setStretchFactor(0, 5);
  splitter->setStretchFactor(1, 2);
  setCentralWidget(splitter);

  setupRos();

  // 5ms 마다 쌓인 ROS 메시지 콜백을 처리. Ctrl+C 등으로 ROS 가 종료되면 창도 닫는다
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
  qApp->installEventFilter(this);   // 모든 키 입력이 먼저 eventFilter() 를 거치게 한다

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
  // 왼쪽 열: 카메라/하드웨어, 오른쪽 열: 비전 인식 결과
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
    psd_bar_[i]->setRange(0, static_cast<int>(psd_max_range_ * 1000));   // 정수만 받으므로 mm 단위
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

// 토픽 이름 + 램프 한 줄을 추가하고, 램프를 key 로 stats_ 에 등록
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

  // 모드 강제 변경 (버튼을 누르면 control_mode 를 한 번 발행)
  auto * mode_row = new QHBoxLayout;
  mode_row->addWidget(new QLabel("control_mode:"));
  using M = interfaces::msg::ControlMode;
  for (uint8_t m : {M::STOP, M::LANE, M::AVOID, M::PARKING, M::MANUAL}) {
    auto * btn = new QPushButton(modeName(m));
    btn->setFocusPolicy(Qt::NoFocus);   // 버튼이 포커스를 가져가 Space 로 눌리는 일이 없도록
    connect(btn, &QPushButton::clicked, this, [this, m] {sendControlMode(m);});
    mode_row->addWidget(btn);
  }
  layout->addLayout(mode_row);

  // 수동 주행
  manual_enable_ = new QCheckBox("수동 주행 (cmd_vel 발행)  — W/A/S/D, Space=정지");
  manual_enable_->setFocusPolicy(Qt::NoFocus);
  // 체크를 끄면 바로 정지 명령을 한 번 보낸다
  connect(manual_enable_, &QCheckBox::toggled, this, [this](bool on) {
    pressed_keys_.clear();
    log(on ? "수동 주행 ON: cmd_vel 발행 시작" : "수동 주행 OFF");
    if (!on) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      was_moving_ = false;
    }
  });
  layout->addWidget(manual_enable_);

  // 속도 조절: 슬라이더와 숫자칸이 서로 연동된다. 주행 중에 바꿔도 다음 송신(0.1초)부터 바로 반영
  auto * speed_grid = new QGridLayout;
  auto make_speed = [this, speed_grid](int row, const QString & name, const QString & unit,
      double init, double max, double step) {
      auto * spin = new QDoubleSpinBox;
      spin->setRange(0.0, max);
      spin->setDecimals(2);
      spin->setSingleStep(step);
      spin->setValue(std::min(init, max));
      spin->setSuffix(" " + unit);
      // 슬라이더는 정수만 다루므로 step 단위 칸 수로 바꿔서 쓴다 (0.01 m/s -> 1칸)
      auto * slider = new QSlider(Qt::Horizontal);
      slider->setRange(0, static_cast<int>(std::lround(max / step)));
      slider->setValue(static_cast<int>(std::lround(spin->value() / step)));
      connect(slider, &QSlider::valueChanged, spin, [spin, step](int v) {spin->setValue(v * step);});
      connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), slider,
        [slider, step](double v) {
          const QSignalBlocker block(slider);   // 슬라이더 -> 숫자칸 -> 슬라이더 되먹임 방지
          slider->setValue(static_cast<int>(std::lround(v / step)));
        });
      connect(spin, &QDoubleSpinBox::editingFinished, this, [this, name, spin] {
        log(QString("%1 = %2").arg(name).arg(spin->value(), 0, 'f', 2));
      });
      speed_grid->addWidget(new QLabel(name), row, 0);
      speed_grid->addWidget(slider, row, 1);
      speed_grid->addWidget(spin, row, 2);
      return spin;
    };
  lin_speed_ = make_speed(0, "선속도 (W/S)", "m/s", lin_speed_init_, lin_speed_max_, 0.01);
  ang_speed_ = make_speed(1, "각속도 (A/D)", "rad/s", ang_speed_init_, ang_speed_max_, 0.1);
  speed_grid->setColumnStretch(1, 1);
  layout->addLayout(speed_grid);

  // 화면 버튼 (누르고 있는 동안 이동)
  // 버튼을 누르면 키보드로 그 키를 누른 것과 똑같이 pressed_keys_ 에 넣는다
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

// 모든 구독/발행 생성. 구독 콜백은 받은 값을 해당 라벨에 표시하고 touch() 로 수신 기록만 남긴다
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
      const float v[3] = {msg->left, msg->front, msg->right};   // [m], 막대는 mm 단위
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
      // 다이나믹셀 하드웨어 에러가 있으면 빨간 글씨로 에러 코드(16진수) 표시
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
  // cmd_vel 은 stm_bridge 가 받아 바로 바퀴로 보낸다 (수동 주행 체크 시에만 발행)
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
  mode_pub_ = node_->create_publisher<interfaces::msg::ControlMode>("control_mode", 10);
}

// 토픽 key 를 방금 받았다고 기록 (처음 받으면 로그, 램프는 즉시 초록)
void MainWindow::touch(const QString & key)
{
  auto & s = stats_[key];
  if (!s.last.isValid()) {
    log(QString("첫 수신: %1").arg(key));
  }
  s.last.start();
  ++s.count;
  // 매 메시지마다 setStyleSheet 하면 느리므로 색이 바뀔 때만
  if (s.lamp && !s.lamp->styleSheet().contains("green")) {
    s.lamp->setStyleSheet("color:green;");
  }
}

void MainWindow::refreshLamps()
{
  // 1초마다 호출 -> count 가 곧 Hz
  for (auto & [key, s] : stats_) {
    if (!s.lamp || !s.last.isValid()) {
      continue;   // 한 번도 안 받은 토픽은 회색 그대로
    }
    const bool stale = s.last.elapsed() > kStaleMs;
    s.lamp->setStyleSheet(stale ? "color:#d32f2f;" : "color:green;");
    s.lamp->setText(QString("●  %1 Hz").arg(s.count));
    s.count = 0;
  }
}

// 10Hz 로 호출: 눌린 키 조합으로 cmd_vel 을 만들어 발행
//   W/S = 전진/후진, A/D = 좌회전/우회전(w 양수 = 왼쪽), 동시에 누르면 합쳐진다 (W+A = 왼쪽으로 돌며 전진)
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

// 모드 버튼: control_mode 를 한 번 발행 (mission_state 에 "turtle_gui" 를 넣어 GUI 가 보낸 것임을 표시)
void MainWindow::sendControlMode(uint8_t mode)
{
  interfaces::msg::ControlMode msg;
  msg.header.stamp = node_->now();
  msg.mode = mode;
  msg.mission_state = "turtle_gui";
  mode_pub_->publish(msg);
  log(QString("control_mode 발행: %1").arg(modeName(mode)));
}

// 앱 전체 키 입력 가로채기. W/A/S/D/Space 는 여기서 처리하고 true 를 반환해 다른 위젯에 안 넘긴다
bool MainWindow::eventFilter(QObject * obj, QEvent * event)
{
  if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
    auto * ke = static_cast<QKeyEvent *>(event);
    const int key = ke->key();
    if (key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D) {
      // 키를 꾹 누르면 OS 가 Press/Release 를 반복해서 보내는데(auto repeat), 그건 무시한다
      if (!ke->isAutoRepeat()) {
        if (event->type() == QEvent::KeyPress) {
          pressed_keys_.insert(key);
        } else {
          pressed_keys_.erase(key);
        }
      }
      return true;
    }
    // Space: 즉시 정지 (수동 주행 체크와 상관없이 정지 명령 발행)
    if (key == Qt::Key_Space) {
      if (event->type() == QEvent::KeyPress && !ke->isAutoRepeat()) {
        pressed_keys_.clear();
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        was_moving_ = false;
      }
      return true;
    }
  }
  return QMainWindow::eventFilter(obj, event);   // 나머지 키는 원래대로 처리
}

void MainWindow::log(const QString & text)
{
  log_view_->appendPlainText(
    QDateTime::currentDateTime().toString("hh:mm:ss  ") + text);
}

// 카메라 화면 하나 (제목 상자 + 영상 + 정보 줄)
QWidget * MainWindow::buildCameraView(CameraView & view, const QString & title)
{
  auto * box = new QGroupBox(title);
  auto * layout = new QVBoxLayout(box);
  layout->setContentsMargins(4, 4, 4, 4);
  view.image = new QLabel("수신 대기 중\n" + view.topic);
  view.image->setAlignment(Qt::AlignCenter);
  view.image->setMinimumSize(240, 180);
  // Ignored: 들어온 영상 크기에 맞춰 라벨이 커지지 않고, 레이아웃이 정한 크기를 따른다
  view.image->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  view.image->setStyleSheet("background:#202020; color:#aaaaaa;");
  view.info = new QLabel("-");
  layout->addWidget(view.image, 1);
  layout->addWidget(view.info);
  view.fps_timer.start();
  return box;
}

// 카메라 토픽 구독. 압축(JPEG) 이면 Qt 가 바로 디코딩하고, 아니면 raw 픽셀을 QImage 로 바꾼다
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
        // QImage(data, ...) 는 메시지 메모리를 빌려 쓰기만 하므로 .copy()/.rgbSwapped() 로 복사본을 만든다
        // (콜백이 끝나면 msg 가 사라지기 때문)
        const auto * data = msg->data.data();
        const int w = static_cast<int>(msg->width), h = static_cast<int>(msg->height);
        const int step = static_cast<int>(msg->step);
        QImage img;
        if (msg->encoding == "rgb8") {
          img = QImage(data, w, h, step, QImage::Format_RGB888).copy();
        } else if (msg->encoding == "bgr8") {
          img = QImage(data, w, h, step, QImage::Format_RGB888).rgbSwapped();   // B, R 순서 바꾸기
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

// 프레임 하나를 화면에 표시하고 fps 갱신 (1초마다 받은 프레임 수로 계산)
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
  // 라벨 크기에 맞춰 비율을 유지하며 축소/확대
  view.image->setPixmap(QPixmap::fromImage(image).scaled(
      view.image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
