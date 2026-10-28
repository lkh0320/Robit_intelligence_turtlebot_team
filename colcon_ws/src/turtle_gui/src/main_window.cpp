// MainWindow 구현: 화면 구성, ROS 구독/발행, 키보드 수동 주행
#include "turtle_gui/main_window.hpp"

#include <algorithm>
#include <utility>

#include <QAbstractSpinBox>
#include <QApplication>
#include <QCheckBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QScreen>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>

#include "turtle_gui/param_panel.hpp"

namespace
{
constexpr int kStaleMs = 1000;   // 이 시간 이상 수신이 없으면 빨간 램프 / 인식 결과 "-"

// 작은 둥근 꼬리표 (색 배경 + 흰 글씨). 값이 없으면 회색
QString chip(const QString & text, const QString & color)
{
  return QString("<span style='background:%1; color:white; font-weight:bold;'>&nbsp;%2&nbsp;</span>")
         .arg(color, text);
}

QString grayChip()
{
  return "<span style='color:#9e9e9e;'>-</span>";
}

QString lightChip(uint8_t s)
{
  using M = interfaces::msg::TrafficLight;
  switch (s) {
    case M::RED: return chip("RED", "#d32f2f");
    case M::YELLOW: return chip("YELLOW", "#f9a825");
    case M::GREEN: return chip("GREEN", "#2e7d32");
    default: return grayChip();
  }
}

QString signText(uint8_t s)
{
  using M = interfaces::msg::Sign;
  switch (s) {
    case M::LEFT: return "LEFT";
    case M::RIGHT: return "RIGHT";
    case M::CONSTRUCTION: return "공사";
    case M::PARKING: return "주차";
    default: return "";
  }
}

// STM32 상태 (DxlState::state). 정상 주행이면 초록
QString stmState(uint8_t st)
{
  using M = interfaces::msg::DxlState;
  switch (st) {
    case M::STATE_STOP_SW: return "<span style='color:#f9a825'>S1 꺼짐</span>";
    case M::STATE_ROS: return "<span style='color:#2e7d32'>ROS 주행</span>";
    case M::STATE_ESTOP: return "<b style='color:#d32f2f'>비상 정지 (S2)</b>";
    case M::STATE_NO_CMD: return "<span style='color:#757575'>명령 없음</span>";
    case M::STATE_NOT_READY: return "<b style='color:#d32f2f'>모터 준비 안 됨</b>";
    case M::STATE_MANUAL: return "<span style='color:#1565c0'>스위치 테스트</span>";
    default: return QString("? (%1)").arg(st);
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

QString modeColor(uint8_t m)
{
  using M = interfaces::msg::ControlMode;
  switch (m) {
    case M::STOP: return "#d32f2f";
    case M::LANE: return "#2e7d32";
    case M::AVOID: return "#ef6c00";
    case M::PARKING: return "#1565c0";
    default: return "#616161";
  }
}

// 흰 바탕 카드 (제목 + 내용). 상자 테두리 대신 옅은 배경으로 구역을 나눈다
QFrame * card(const QString & title, QLayout * body)
{
  auto * frame = new QFrame;
  frame->setObjectName("card");
  frame->setStyleSheet("#card { background:#ffffff; border:1px solid #dde1e6; border-radius:8px; }");
  auto * v = new QVBoxLayout(frame);
  v->setContentsMargins(10, 8, 10, 10);
  v->setSpacing(6);
  if (!title.isEmpty()) {
    auto * t = new QLabel(title);
    t->setStyleSheet("color:#6b7280; font-size:11px; font-weight:bold;");
    v->addWidget(t);
  }
  v->addLayout(body);
  return frame;
}
}  // namespace

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent)
: QMainWindow(parent), node_(std::move(node))
{
  // 파라미터 (ros2 run turtle_gui turtle_gui --ros-args -p image_topic:=... 로 바꿀 수 있다)
  psd_warn_ = node_->declare_parameter("psd_warn", 0.25);   // task_planner psd_obstacle_threshold 와 같게
  // 키보드 수동 주행 속도 시작값 (GUI 에서 바로 바꿀 수 있다)
  lin_speed_init_ = node_->declare_parameter("linear_speed", 0.10);
  ang_speed_init_ = node_->declare_parameter("angular_speed", 1.5);

  // 카메라 화면 3개가 볼 토픽
  // grid_drive (격자 계획 주행): 원본 + 도로 마스크 | 계획 지도 + 경로 -> 가로로 긴 영상이라 위에 크게
  cams_[0].key = "cam_grid";
  cams_[0].topic = QString::fromStdString(
    node_->declare_parameter("grid_image_topic", std::string("grid_drive/debug/compressed")));
  cams_[1].key = "cam_raw";
  cams_[1].topic = QString::fromStdString(
    node_->declare_parameter("image_topic", std::string("image_raw/compressed")));
  cams_[2].key = "cam_object";
  cams_[2].topic = QString::fromStdString(
    node_->declare_parameter("object_image_topic", std::string("vision/object_debug/compressed")));

  setWindowTitle("TurtleBot GUI");
  setStyleSheet("QMainWindow { background:#f3f4f6; }");
  // 기본 1600x1000 (16:10), 화면이 더 작으면 비율을 유지한 채 화면에 맞춤
  const QSize avail = QGuiApplication::primaryScreen()->availableGeometry().size();
  QSize size(1600, 1000);
  if (size.width() > avail.width() || size.height() > avail.height()) {
    size.scale(avail, Qt::KeepAspectRatio);
  }
  resize(size);

  // 왼쪽: 카메라 + 로그
  //   격자 계획 주행 (grid_drive)
  //   원본 | 객체 인식
  // (카메라 밝기/노출은 GUI 가 아니라 Jetson 에서 v4l2-ctl 로 설정: vision_bringup/camera/README.md)
  auto * left = new QWidget;
  auto * left_layout = new QVBoxLayout(left);
  left_layout->setContentsMargins(8, 8, 4, 8);
  auto * cam_grid = new QGridLayout;
  cam_grid->setSpacing(8);
  cam_grid->addWidget(buildCameraView(cams_[0], "격자 계획 주행"), 0, 0, 1, 2);
  cam_grid->addWidget(buildCameraView(cams_[1], "원본"), 1, 0);
  cam_grid->addWidget(buildCameraView(cams_[2], "객체 인식"), 1, 1);
  cam_grid->setRowStretch(0, 3);
  cam_grid->setRowStretch(1, 2);
  // 아래쪽 로그창 (최대 500줄, 오래된 줄부터 지워짐)
  log_view_ = new QPlainTextEdit;
  log_view_->setReadOnly(true);
  log_view_->setMaximumBlockCount(500);
  log_view_->setMaximumHeight(90);
  log_view_->setStyleSheet("QPlainTextEdit { background:#ffffff; border:1px solid #dde1e6; "
    "border-radius:6px; font-family:monospace; font-size:11px; }");
  left_layout->addLayout(cam_grid, 1);
  left_layout->addWidget(log_view_);

  // 오른쪽: 상태 + 조종 (위, 고정 높이) / 노드 파라미터 탭 (아래, 남는 높이 전부)
  auto * right = new QWidget;
  auto * right_layout = new QVBoxLayout(right);
  right_layout->setContentsMargins(4, 8, 8, 8);
  right_layout->setSpacing(8);
  right_layout->addWidget(buildStatusPanel());
  right_layout->addWidget(buildControlPanel());
  params_ = new ParamPanel(node_);
  connect(params_, &ParamPanel::message, this, &MainWindow::log);
  right_layout->addWidget(params_, 1);

  // 좌우 경계를 마우스로 끌어 크기 조절 가능. 처음 비율은 3 : 2
  auto * splitter = new QSplitter;
  splitter->addWidget(left);
  splitter->addWidget(right);
  splitter->setStretchFactor(0, 3);
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
  auto * body = new QVBoxLayout;
  body->setSpacing(8);

  // 1) 제어 모드 (크게) + 토픽 수신 점들
  auto * top = new QHBoxLayout;
  mode_label_ = new QLabel("<span style='color:#9e9e9e; font-size:20px; font-weight:bold;'>-</span>");
  top->addWidget(mode_label_, 1);
  auto * lamps = new QHBoxLayout;
  lamps->setSpacing(6);
  addLamp(lamps, "cam_raw", "카메라", cams_[1].topic);
  addLamp(lamps, "cam_grid", "grid", cams_[0].topic);
  addLamp(lamps, "psd", "PSD", "psd");
  addLamp(lamps, "dxl", "STM", "dxl_state");
  addLamp(lamps, "mode", "모드", "control_mode");
  top->addLayout(lamps);
  body->addLayout(top);

  // 2) 로봇: 배터리 / STM 상태 / 모터 이상
  robot_label_ = new QLabel("<span style='color:#9e9e9e'>STM 수신 대기</span>");
  body->addWidget(robot_label_);

  // 3) PSD 세 칸 (가까우면 빨강) + 바퀴 속도 / cmd_vel
  auto * psd_row = new QHBoxLayout;
  psd_row->setSpacing(6);
  const char * names[3] = {"왼쪽", "앞", "오른쪽"};
  for (int i = 0; i < 3; ++i) {
    auto * cell = new QVBoxLayout;
    cell->setSpacing(0);
    auto * name = new QLabel(QString("PSD %1").arg(names[i]));
    name->setAlignment(Qt::AlignCenter);
    name->setStyleSheet("color:#6b7280; font-size:10px;");
    psd_value_[i] = new QLabel("-");
    psd_value_[i]->setAlignment(Qt::AlignCenter);
    psd_value_[i]->setStyleSheet("font-size:16px; font-weight:bold; color:#9e9e9e;");
    cell->addWidget(name);
    cell->addWidget(psd_value_[i]);
    auto * box = new QFrame;
    box->setStyleSheet("QFrame { background:#f3f4f6; border-radius:6px; }");
    box->setLayout(cell);
    psd_row->addWidget(box, 1);
  }
  body->addLayout(psd_row);
  motor_label_ = new QLabel("<span style='color:#9e9e9e'>모터 -</span>");
  body->addWidget(motor_label_);

  // 4) 비전 인식: 신호등 / 표지판 / 차단바 꼬리표 한 줄 (1초 동안 안 오면 "-")
  auto * vision = new QHBoxLayout;
  auto add_chip = [vision](const QString & name) {
      auto * n = new QLabel(name);
      n->setStyleSheet("color:#6b7280;");
      auto * v = new QLabel(grayChip());
      vision->addWidget(n);
      vision->addWidget(v);
      vision->addSpacing(10);
      return v;
    };
  light_chip_ = add_chip("신호등");
  sign_chip_ = add_chip("표지판");
  barrier_chip_ = add_chip("차단바");
  vision->addStretch();
  body->addLayout(vision);

  return card("상태", body);
}

// 수신 점 하나 ("● 카메라"). 툴팁에 토픽 이름과 Hz
void MainWindow::addLamp(QHBoxLayout * row, const QString & key, const QString & text, const QString & topic)
{
  auto * lamp = new QLabel(QString("<span style='color:#bdbdbd'>●</span> %1").arg(text));
  lamp->setStyleSheet("color:#6b7280; font-size:11px;");
  lamp->setToolTip(topic + ": 수신 없음");
  row->addWidget(lamp);
  stats_[key].lamp = lamp;
  stats_[key].topic = topic;
}

QWidget * MainWindow::buildControlPanel()
{
  auto * body = new QVBoxLayout;
  body->setSpacing(8);

  // 버튼 두 개만: 정지 / 자율주행 시작
  auto * buttons = new QHBoxLayout;
  buttons->setSpacing(8);
  auto make_button = [buttons](const QString & text, const QString & color, const QString & hover) {
      auto * b = new QPushButton(text);
      b->setFocusPolicy(Qt::NoFocus);   // 버튼이 포커스를 가져가 Space 로 눌리는 일이 없도록
      b->setMinimumHeight(48);
      b->setCursor(Qt::PointingHandCursor);
      b->setStyleSheet(QString(
        "QPushButton { background:%1; color:white; font-size:16px; font-weight:bold; border:none; "
        "border-radius:8px; } QPushButton:hover { background:%2; } QPushButton:pressed { padding-top:2px; }")
        .arg(color, hover));
      buttons->addWidget(b, 1);
      return b;
    };
  // 정지: control_mode STOP (task_planner / grid_drive 멈춤) + cmd_vel 0
  auto * stop_btn = make_button("■  STOP", "#d32f2f", "#b71c1c");
  stop_btn->setToolTip("control_mode STOP + cmd_vel 0  (Space 는 cmd_vel 0 만)");
  connect(stop_btn, &QPushButton::clicked, this, &MainWindow::sendStop);
  // 자율주행 시작: task_planner 가 신호등 대기(WAIT_LIGHT)부터 미션을 진행한다
  auto * start_btn = make_button("▶  자율주행 시작", "#2e7d32", "#1b5e20");
  start_btn->setToolTip("mission_start 발행: 신호등 대기부터 미션 시작");
  connect(start_btn, &QPushButton::clicked, this, &MainWindow::sendMissionStart);
  body->addLayout(buttons);

  // 키보드 수동 주행 (W/A/S/D, Space = 정지). 체크해야 cmd_vel 을 보낸다
  auto * manual = new QHBoxLayout;
  manual_enable_ = new QCheckBox("키보드 주행 (WASD)");
  manual_enable_->setFocusPolicy(Qt::NoFocus);
  // 체크를 끄면 바로 정지 명령을 한 번 보낸다
  connect(manual_enable_, &QCheckBox::toggled, this, [this](bool on) {
    pressed_keys_.clear();
    log(on ? "키보드 주행 ON: cmd_vel 발행 시작" : "키보드 주행 OFF");
    if (!on) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      was_moving_ = false;
    }
  });
  manual->addWidget(manual_enable_);
  manual->addStretch();
  auto make_speed = [manual](double init, double max, double step, const QString & unit) {
      auto * spin = new QDoubleSpinBox;
      spin->setRange(0.0, max);
      spin->setDecimals(2);
      spin->setSingleStep(step);
      spin->setValue(std::min(init, max));
      spin->setSuffix(" " + unit);
      spin->setFocusPolicy(Qt::ClickFocus);
      manual->addWidget(spin);
      return spin;
    };
  // 상한은 stm_bridge max_wheel_speed / 펌웨어 MAX_WHEEL_MPS 가 따로 막는다
  lin_speed_ = make_speed(lin_speed_init_, 1.0, 0.01, "m/s");
  ang_speed_ = make_speed(ang_speed_init_, 10.0, 0.1, "rad/s");
  body->addLayout(manual);

  return card("조종", body);
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
      const float v[3] = {msg->left, msg->front, msg->right};   // [m]
      for (int i = 0; i < 3; ++i) {
        psd_value_[i]->setText(QString::number(v[i], 'f', 2));
        psd_value_[i]->setStyleSheet(QString("font-size:16px; font-weight:bold; color:%1;")
          .arg(v[i] < psd_warn_ ? "#d32f2f" : "#111827"));
      }
    });

  dxl_sub_ = node_->create_subscription<interfaces::msg::DxlState>(
    "dxl_state", qos, [this](interfaces::msg::DxlState::ConstSharedPtr msg) {
      touch("dxl");
      // 배터리 · STM 상태 · 모터 (정상이면 "모터 OK" 만, 이상이면 무엇이 문제인지)
      QString motor;
      if (msg->motor_count != 2 || !msg->left_torque || !msg->right_torque) {
        motor = QString("<b style='color:#d32f2f'>모터 %1개 (L%2 R%3)</b>").arg(msg->motor_count)
          .arg(msg->left_torque ? "" : " 토크OFF").arg(msg->right_torque ? "" : " 토크OFF");
      } else {
        motor = "<span style='color:#2e7d32'>모터 OK</span>";
      }
      if (msg->left_error || msg->right_error) {
        motor += QString("  <b style='color:#d32f2f'>ERR L=0x%1 R=0x%2</b>")
          .arg(msg->left_error, 2, 16, QChar('0')).arg(msg->right_error, 2, 16, QChar('0'));
      }
      robot_label_->setText(QString("<b style='font-size:14px'>%1 V</b> &nbsp;·&nbsp; %2 &nbsp;·&nbsp; %3")
        .arg(msg->voltage, 0, 'f', 1).arg(stmState(msg->state)).arg(motor));
      wheel_text_ = QString("바퀴 L %1  R %2 m/s").arg(msg->left_velocity, 0, 'f', 2)
        .arg(msg->right_velocity, 0, 'f', 2);
      refreshMotor();
    });

  // 실제로 로봇에 들어가는 cmd_vel (GUI가 보낸 것 포함)
  cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    cmd_topic, 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
      touch("cmd");
      cmd_text_ = QString("명령 v %1  w %2").arg(msg->linear.x, 0, 'f', 2).arg(msg->angular.z, 0, 'f', 2);
      refreshMotor();
    });

  mode_sub_ = node_->create_subscription<interfaces::msg::ControlMode>(
    "control_mode", 10, [this](interfaces::msg::ControlMode::ConstSharedPtr msg) {
      touch("mode");
      mode_label_->setText(QString("<span style='color:%1; font-size:20px; font-weight:bold;'>%2</span>"
        "&nbsp;&nbsp;<span style='color:#6b7280; font-size:12px;'>%3</span>")
        .arg(modeColor(msg->mode), modeName(msg->mode),
        QString::fromStdString(msg->mission_state).toHtmlEscaped()));
    });

  // --- vision ---
  light_sub_ = node_->create_subscription<interfaces::msg::TrafficLight>(
    "traffic_light", qos, [this](interfaces::msg::TrafficLight::ConstSharedPtr m) {
      touch("light");
      light_chip_->setText(lightChip(m->state));
    });
  sign_sub_ = node_->create_subscription<interfaces::msg::Sign>(
    "sign", qos, [this](interfaces::msg::Sign::ConstSharedPtr m) {
      touch("sign");
      const QString name = signText(m->type);
      sign_chip_->setText(name.isEmpty() ? grayChip() :
        chip(QString("%1 %2").arg(name).arg(m->confidence, 0, 'f', 2), "#1565c0"));
    });
  barrier_sub_ = node_->create_subscription<interfaces::msg::Barrier>(
    "barrier", qos, [this](interfaces::msg::Barrier::ConstSharedPtr m) {
      touch("barrier");
      using B = interfaces::msg::Barrier;
      barrier_chip_->setText(!m->detected ? grayChip() :
        m->state == B::CLOSED ? chip("닫힘", "#d32f2f") :
        m->state == B::OPEN ? chip("열림", "#2e7d32") : chip("?", "#757575"));
    });

  // --- 송신 ---
  // cmd_vel 은 stm_bridge 가 받아 바로 바퀴로 보낸다 (키보드 주행 체크 시 / 정지 때만 발행)
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
  mode_pub_ = node_->create_publisher<interfaces::msg::ControlMode>("control_mode", 10);
  start_pub_ = node_->create_publisher<std_msgs::msg::Empty>("mission_start", 10);
}

void MainWindow::refreshMotor()
{
  QStringList parts;
  if (!wheel_text_.isEmpty()) {parts << wheel_text_;}
  if (!cmd_text_.isEmpty()) {parts << cmd_text_;}
  motor_label_->setText(parts.isEmpty() ? "<span style='color:#9e9e9e'>모터 -</span>" :
    "<span style='font-family:monospace;'>" + parts.join("&nbsp;&nbsp;│&nbsp;&nbsp;") + "</span>");
}

// 토픽 key 를 방금 받았다고 기록 (처음 받으면 로그)
void MainWindow::touch(const QString & key)
{
  auto & s = stats_[key];
  if (!s.last.isValid()) {
    log(QString("첫 수신: %1").arg(key));
  }
  s.last.start();
  ++s.count;
}

bool MainWindow::fresh(const QString & key) const
{
  const auto it = stats_.find(key);
  return it != stats_.end() && it->second.last.isValid() && it->second.last.elapsed() <= kStaleMs;
}

void MainWindow::refreshLamps()
{
  // 1초마다 호출 -> count 가 곧 Hz
  for (auto & [key, s] : stats_) {
    if (s.lamp && s.last.isValid()) {
      const bool stale = s.last.elapsed() > kStaleMs;
      QString text = s.lamp->text();
      text.replace(QRegularExpression("color:#[0-9a-f]+"), stale ? "color:#d32f2f" : "color:#2e7d32");
      s.lamp->setText(text);
      s.lamp->setToolTip(QString("%1: %2 Hz").arg(s.topic).arg(s.count));
    }
    s.count = 0;
  }
  // 인식 결과가 끊기면 마지막 값을 계속 보여주지 않고 "-"
  if (!fresh("light")) {light_chip_->setText(grayChip());}
  if (!fresh("sign")) {sign_chip_->setText(grayChip());}
  if (!fresh("barrier")) {barrier_chip_->setText(grayChip());}
  if (!fresh("psd")) {
    for (auto * l : psd_value_) {
      l->setStyleSheet("font-size:16px; font-weight:bold; color:#9e9e9e;");
    }
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

// STOP 버튼: control_mode STOP 을 한 번 발행 (mission_state "turtle_gui" = GUI 가 보낸 것) + cmd_vel 0
//   task_planner 는 GUI STOP 을 받으면 미션을 멈춘다. 다시 가려면 자율주행 시작
void MainWindow::sendStop()
{
  manual_enable_->setChecked(false);
  pressed_keys_.clear();
  interfaces::msg::ControlMode msg;
  msg.header.stamp = node_->now();
  msg.mode = interfaces::msg::ControlMode::STOP;
  msg.mission_state = "turtle_gui";
  mode_pub_->publish(msg);
  cmd_pub_->publish(geometry_msgs::msg::Twist());
  was_moving_ = false;
  log("STOP: control_mode STOP + cmd_vel 0");
}

// 자율주행 시작 버튼: task_planner 에 mission_start 를 보낸다 (처음부터 다시 시작, GUI 오버라이드 해제)
//   키보드 주행 cmd_vel 과 섞이지 않도록 키보드 주행 체크는 끈다
void MainWindow::sendMissionStart()
{
  manual_enable_->setChecked(false);
  start_pub_->publish(std_msgs::msg::Empty());
  log("mission_start 발행: 자율주행 시작 (신호등 대기부터)");
}

// 앱 전체 키 입력 가로채기. W/A/S/D/Space 는 여기서 처리하고 true 를 반환해 다른 위젯에 안 넘긴다
//   단, 파라미터 값/검색칸 같은 글자 입력칸에 포커스가 있으면 그 칸에 글자를 넘긴다
bool MainWindow::eventFilter(QObject * obj, QEvent * event)
{
  if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
    const QWidget * focus = QApplication::focusWidget();
    if (qobject_cast<const QLineEdit *>(focus) || qobject_cast<const QAbstractSpinBox *>(focus)) {
      return QMainWindow::eventFilter(obj, event);
    }
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
    // Space: 즉시 정지 (키보드 주행 체크와 상관없이 cmd_vel 0 발행)
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

// 카메라 화면 하나 (제목 줄 + 영상)
QWidget * MainWindow::buildCameraView(CameraView & view, const QString & title)
{
  auto * frame = new QFrame;
  frame->setObjectName("cam");
  frame->setStyleSheet("#cam { background:#ffffff; border:1px solid #dde1e6; border-radius:8px; }");
  auto * layout = new QVBoxLayout(frame);
  layout->setContentsMargins(6, 4, 6, 6);
  layout->setSpacing(4);
  auto * head = new QHBoxLayout;
  auto * t = new QLabel(title);
  t->setStyleSheet("font-weight:bold; color:#374151;");
  view.info = new QLabel("");
  view.info->setStyleSheet("color:#9ca3af; font-size:11px;");
  view.info->setToolTip(view.topic);
  head->addWidget(t);
  head->addStretch();
  head->addWidget(view.info);
  view.image = new QLabel("수신 대기 중\n" + view.topic);
  view.image->setAlignment(Qt::AlignCenter);
  view.image->setMinimumSize(200, 120);
  // Ignored: 들어온 영상 크기에 맞춰 라벨이 커지지 않고, 레이아웃이 정한 크기를 따른다
  view.image->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  view.image->setStyleSheet("background:#1f2328; color:#8b949e; border-radius:4px;");
  layout->addLayout(head);
  layout->addWidget(view.image, 1);
  view.fps_timer.start();
  return frame;
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
}

// 프레임 하나를 화면에 표시하고 fps 갱신 (1초마다 받은 프레임 수로 계산)
void MainWindow::onFrame(CameraView & view, const QImage & image)
{
  touch(view.key);
  ++view.frames;
  if (view.fps_timer.elapsed() >= 1000) {
    view.fps = view.frames * 1000.0 / view.fps_timer.restart();
    view.frames = 0;
    view.info->setText(QString("%1x%2  %3 fps").arg(image.width()).arg(image.height())
      .arg(view.fps, 0, 'f', 1));
  }
  // 라벨 크기에 맞춰 비율을 유지하며 축소/확대
  view.image->setPixmap(QPixmap::fromImage(image).scaled(
      view.image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
