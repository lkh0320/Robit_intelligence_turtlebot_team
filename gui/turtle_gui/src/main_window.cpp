// MainWindow 구현: 화면 표시 + 버튼/키보드 입력 처리
//  - 생성자: 위젯 초기 설정, signal/slot 연결, 타이머 시작
//  - on*(): QNode 에서 받은 데이터를 화면에 표시
//  - 수동 주행: 키/버튼 상태를 drive_[] 에 기록 → 0.1초 타이머(sendManualCmd)가 명령 발행
//  - 파라미터 탭: 표에 값 표시 → 수정된 칸만 모아서 적용
#include "turtle_gui/main_window.hpp"

#include <algorithm>

#include <QCloseEvent>
#include <QDateTime>
#include <QHeaderView>
#include <QKeyEvent>
#include <QScrollBar>

#include "ui_main_window.h"                                         // ui/main_window.ui 로부터 빌드 시 자동 생성되는 헤더

namespace
{
// 파라미터 탭 콤보박스 기본 목록 ([노드 찾기]를 누르면 실제 실행 중인 노드로 바뀜)
const QStringList kDefaultParamNodes = {"/camera_node", "/vision_node", "/control_node", "/stm32_bridge_node"};
// 표의 값 칸에 화면에 안 보이는 데이터를 같이 저장할 때 쓰는 키
const int kParamTypeRole = Qt::UserRole;                              // 파라미터 타입 (적용할 때 문자열 → 원래 타입 변환용)
const int kParamOriginalRole = Qt::UserRole + 1;                      // 불러왔을 때의 원래 값 (수정 여부 판단용)
}  // namespace

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);                                                  // .ui 에 그린 위젯들을 실제로 생성
  ui->plainTextEdit_log->setMaximumBlockCount(500);                   // 로그는 최근 500줄만 유지

  ui->comboBox_param_node->addItems(kDefaultParamNodes);
  ui->tableWidget_params->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  ui->tableWidget_params->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  ui->tableWidget_params->horizontalHeader()->setStretchLastSection(true);
  ui->tableWidget_params->verticalHeader()->setVisible(false);

  // 연결 램프 전부 회색(아직 수신 없음)으로 시작
  for (QLabel * lamp : {ui->label_lamp_camera, ui->label_lamp_vision, ui->label_lamp_psd,
                        ui->label_lamp_motor, ui->label_lamp_robot, ui->label_lamp_stm32}) {
    lamp->setMinimumHeight(28);
    setLamp(lamp, 0);
  }

  // QNode -> 화면
  // connect(보내는 객체, 어떤 signal, 받는 객체, 실행할 함수): signal 이 emit 되면 함수가 메인 스레드에서 실행됨
  connect(&qnode, &QNode::imageReceived, this, &MainWindow::onImage);
  connect(&qnode, &QNode::psdReceived, this, &MainWindow::onPsd);
  connect(&qnode, &QNode::motorReceived, this, &MainWindow::onMotor);
  connect(&qnode, &QNode::robotStateReceived, this, &MainWindow::onRobotState);
  connect(&qnode, &QNode::visionReceived, this, &MainWindow::onVision);
  connect(&qnode, &QNode::parametersLoaded, this, &MainWindow::onParametersLoaded);
  connect(&qnode, &QNode::parametersApplied, this, [this](const QString & node_name) {  // 적용 후 실제 값을 다시 읽어 표 갱신
    qnode.loadParameters(node_name);
  });
  connect(&qnode, &QNode::logMessage, this, &MainWindow::onLog);
  connect(&qnode, &QNode::rosShutdown, this, &MainWindow::close);

  // 영상 선택 (카메라 원본 / 영상처리 결과). 바꾸면 이전 화면을 지우고 새 토픽 구독
  connect(ui->comboBox_image_source, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
    last_pixmap_ = QPixmap();
    ui->label_image->clear();
    ui->label_image->setText("영상 없음");
    last_rx_[SRC_CAMERA].invalidate();
    qnode.setImageSource(index);
  });

  // 실행, 비상정지, 모드
  connect(ui->pushButton_start, &QPushButton::clicked, this, [this]() { qnode.callRun(true); });
  connect(ui->pushButton_stop, &QPushButton::clicked, this, [this]() { qnode.callRun(false); });
  connect(ui->pushButton_estop, &QPushButton::clicked, this, [this]() {
    clearDrive();                                                     // 비상정지 시 눌려 있던 주행 키도 모두 해제
    qnode.callEstop(true);
  });
  connect(ui->pushButton_estop_release, &QPushButton::clicked, this, [this]() { qnode.callEstop(false); });
  connect(ui->radioButton_manual, &QRadioButton::clicked, this, [this]() { qnode.callSetMode(0); });
  connect(ui->radioButton_auto, &QRadioButton::clicked, this, [this]() { qnode.callSetMode(1); });

  // 수동 주행 버튼 (누르는 동안만 주행)
  // pressed/released 에서 상태만 바꾸고, 실제 명령 발행은 drive_timer_ 가 0.1초마다 한다
  const QList<QPair<QPushButton *, DriveKey>> drive_buttons = {
    {ui->pushButton_w, KEY_W}, {ui->pushButton_a, KEY_A}, {ui->pushButton_s, KEY_S}, {ui->pushButton_d, KEY_D}};
  for (const auto & pair : drive_buttons) {
    const DriveKey key = pair.second;
    connect(pair.first, &QPushButton::pressed, this, [this, key]() { setDrive(key, true); });
    connect(pair.first, &QPushButton::released, this, [this, key]() { setDrive(key, false); });
  }
  // 속도 슬라이더를 움직이면 옆 라벨에 현재 값 표시
  connect(ui->horizontalSlider_linear, &QSlider::valueChanged, this, [this]() {
    ui->label_linear->setText(QString::number(linearSpeed(), 'f', 2) + " m/s");
  });
  connect(ui->horizontalSlider_angular, &QSlider::valueChanged, this, [this]() {
    ui->label_angular->setText(QString::number(angularSpeed(), 'f', 1) + " rad/s");
  });

  // 파라미터 탭
  connect(ui->pushButton_refresh_nodes, &QPushButton::clicked, this, &MainWindow::refreshNodes);
  connect(ui->pushButton_load_params, &QPushButton::clicked, this, &MainWindow::loadParameters);
  connect(ui->pushButton_apply_params, &QPushButton::clicked, this, &MainWindow::applyParameters);
  connect(ui->tableWidget_params, &QTableWidget::itemChanged, this, [](QTableWidgetItem * item) {
    if (item->column() != 2) return;                                  // 0: 이름, 1: 타입, 2: 값 (값 칸만 수정 가능)
    const bool edited = item->text() != item->data(kParamOriginalRole).toString();
    item->setBackground(edited ? QColor("#fff59d") : QColor(Qt::transparent));  // 수정된 칸 노란색
  });
  connect(ui->tabWidget, &QTabWidget::currentChanged, this, [this]() { clearDrive(); });  // 탭을 바꾸면 주행 정지

  connect(&status_timer_, &QTimer::timeout, this, &MainWindow::updateStatus);
  connect(&drive_timer_, &QTimer::timeout, this, &MainWindow::sendManualCmd);
  status_timer_.start(500);                                           // 0.5초마다 램프/fps 갱신
  drive_timer_.start(100);                                            // 0.1초(10 Hz)마다 수동 주행 명령 발행

  qApp->installEventFilter(this);                                     // 어느 위젯에 포커스가 있어도 키 입력 받기
  onLog("GUI 시작 (ROS_DOMAIN_ID 가 로봇과 같은지 확인)");
}

MainWindow::~MainWindow()
{
  qApp->removeEventFilter(this);
  delete ui;
}

void MainWindow::closeEvent(QCloseEvent * event)
{
  if (was_driving_) qnode.publishManualCmd(0.0, 0.0);                 // 주행 중 창을 닫으면 정지 명령
  event->accept();
}

// 키보드 처리: 앱 전체의 키 입력을 여기서 먼저 가로챈다
//  Space = 비상정지, W/A/S/D = 수동 주행 (누르고 있는 동안)
//  return true = "이 키는 처리했음" (다른 위젯으로 안 넘어감), false = 원래대로 전달
bool MainWindow::eventFilter(QObject * watched, QEvent * event)
{
  if (event->type() == QEvent::ApplicationDeactivate) {
    clearDrive();                                                     // 창 밖으로 포커스가 나가면 키 떼짐을 못 받으므로 정지
    return false;
  }
  if (event->type() != QEvent::KeyPress && event->type() != QEvent::KeyRelease) {
    return QMainWindow::eventFilter(watched, event);
  }
  if (ui->tabWidget->currentWidget() == ui->tab_param) return false;  // 파라미터 입력 중에는 키 조종 끔

  auto * key_event = static_cast<QKeyEvent *>(event);
  const bool pressed = event->type() == QEvent::KeyPress;

  if (key_event->key() == Qt::Key_Space) {
    if (pressed && !key_event->isAutoRepeat()) {
      clearDrive();
      qnode.callEstop(true);
    }
    return true;
  }

  DriveKey key;
  switch (key_event->key()) {
    case Qt::Key_W: key = KEY_W; break;
    case Qt::Key_A: key = KEY_A; break;
    case Qt::Key_S: key = KEY_S; break;
    case Qt::Key_D: key = KEY_D; break;
    default: return false;
  }
  if (!key_event->isAutoRepeat()) setDrive(key, pressed);             // 꾹 누를 때 생기는 자동 반복 이벤트는 무시
  return true;
}

// 주행 키 하나의 눌림 상태 저장
void MainWindow::setDrive(DriveKey key, bool pressed)
{
  drive_[key] = pressed;
  QPushButton * buttons[KEY_COUNT] = {ui->pushButton_w, ui->pushButton_a, ui->pushButton_s, ui->pushButton_d};
  buttons[key]->setDown(pressed);                                     // 키보드로 눌러도 버튼이 눌린 모양
}

// 모든 주행 키를 뗀 상태로 (다음 타이머에서 정지 명령이 한 번 나감)
void MainWindow::clearDrive()
{
  for (int i = 0; i < KEY_COUNT; i++) setDrive(static_cast<DriveKey>(i), false);
}

double MainWindow::linearSpeed() const
{
  return ui->horizontalSlider_linear->value() / 100.0;               // 0 ~ 0.30 m/s
}

double MainWindow::angularSpeed() const
{
  return ui->horizontalSlider_angular->value() / 10.0;               // 0 ~ 3.0 rad/s
}

// 0.1초마다 호출. 눌린 키 조합으로 속도를 계산해서 발행
// 예) W + A 동시 → 전진하면서 왼쪽으로 회전
void MainWindow::sendManualCmd()
{
  const int forward = (drive_[KEY_W] ? 1 : 0) - (drive_[KEY_S] ? 1 : 0);  // 1 전진, -1 후진, 0 정지
  const int turn = (drive_[KEY_A] ? 1 : 0) - (drive_[KEY_D] ? 1 : 0);  // 왼쪽 회전이 +

  if (forward != 0 || turn != 0) {
    const double linear = forward * linearSpeed();
    const double angular = turn * angularSpeed();
    qnode.publishManualCmd(linear, angular);
    ui->label_cmd->setText(QString("명령: v %1 m/s, w %2 rad/s")
                             .arg(linear, 0, 'f', 2).arg(angular, 0, 'f', 1));
    was_driving_ = true;
  } else if (was_driving_) {
    qnode.publishManualCmd(0.0, 0.0);                                 // 키를 떼면 정지 명령 한 번
    ui->label_cmd->setText("명령: 정지");
    was_driving_ = false;
  }
}

// 해당 데이터의 "마지막 수신 시각"을 지금으로 갱신
void MainWindow::markReceived(Source source)
{
  last_rx_[source].restart();
}

void MainWindow::setLamp(QLabel * lamp, int state)
{
  static const char * colors[] = {"#9e9e9e", "#43a047", "#e53935"};  // 회색(없음), 초록(정상), 빨강(끊김)
  lamp->setStyleSheet(QString("background-color: %1; color: white; border-radius: 4px; font-weight: bold;")
                        .arg(colors[state]));
}

// 0.5초마다 호출: 마지막 수신 후 1초 이내면 초록, 넘으면 빨강, 한 번도 안 받았으면 회색
void MainWindow::updateStatus()
{
  QLabel * lamps[SRC_COUNT] = {ui->label_lamp_camera, ui->label_lamp_vision, ui->label_lamp_psd,
                               ui->label_lamp_motor, ui->label_lamp_robot};
  for (int i = 0; i < SRC_COUNT; i++) {
    if (!last_rx_[i].isValid()) setLamp(lamps[i], 0);
    else setLamp(lamps[i], last_rx_[i].elapsed() < 1000 ? 1 : 2);
  }

  // STM32 램프는 /robot/state 의 stm32_connected 값 + /robot/state 가 계속 오는지로 판단
  const bool robot_alive = last_rx_[SRC_ROBOT].isValid() && last_rx_[SRC_ROBOT].elapsed() < 1000;
  if (!last_rx_[SRC_ROBOT].isValid()) setLamp(ui->label_lamp_stm32, 0);
  else setLamp(ui->label_lamp_stm32, robot_alive && stm32_connected_ ? 1 : 2);

  ui->label_fps->setText(QString("%1 fps").arg(frame_count_ * 2));   // 0.5초마다 세므로 x2
  frame_count_ = 0;
}

// 영상 한 프레임 표시 (라벨 크기에 맞게 비율을 유지하며 크기 조절)
void MainWindow::onImage(const QImage & image)
{
  markReceived(SRC_CAMERA);
  frame_count_++;
  last_pixmap_ = QPixmap::fromImage(image);
  ui->label_image->setPixmap(
    last_pixmap_.scaled(ui->label_image->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
}

// PSD 3개를 막대(거리)와 raw 값으로 표시
void MainWindow::onPsd(const PsdData & data)
{
  markReceived(SRC_PSD);
  QProgressBar * bars[3] = {ui->progressBar_psd_left, ui->progressBar_psd_center, ui->progressBar_psd_right};
  QLabel * raws[3] = {ui->label_psd_left_raw, ui->label_psd_center_raw, ui->label_psd_right_raw};
  for (int i = 0; i < 3; i++) {
    bars[i]->setValue(qMin(data.mm[i], bars[i]->maximum()));
    bars[i]->setFormat(QString("%1 mm").arg(data.mm[i]));             // 최대값 넘어도 실제 값 표시
    raws[i]->setText(QString("raw %1").arg(data.raw[i]));
  }
}

// 바퀴 속도 표시. 에러 코드가 있으면 16진수로 같이 표시
void MainWindow::onMotor(const MotorData & data)
{
  markReceived(SRC_MOTOR);
  auto text = [](double mps, int error) {
    QString s = QString("%1 m/s").arg(mps, 0, 'f', 3);
    if (error != 0) s += QString("  (에러 0x%1)").arg(error, 2, 16, QChar('0'));
    return s;
  };
  ui->label_motor_left->setText(text(data.left_mps, data.left_error));
  ui->label_motor_right->setText(text(data.right_mps, data.right_error));
}

// 로봇 상태 표시 (모드, 실행, 비상정지, STM32 에러)
void MainWindow::onRobotState(const RobotStateData & data)
{
  markReceived(SRC_ROBOT);
  stm32_connected_ = data.stm32_connected;

  ui->label_mode->setText(data.mode == 0 ? "수동" : "자율");
  ui->label_running->setText(data.running ? "실행 중" : "정지");
  ui->label_estop->setText(data.estop ? "비상정지 걸림" : "정상");
  ui->label_estop->setStyleSheet(data.estop ? "color: #d32f2f; font-weight: bold;" : "");
  ui->label_mcu_error->setText(data.mcu_error_flags == 0
                                 ? "없음"
                                 : QString("0x%1").arg(data.mcu_error_flags, 4, 16, QChar('0')));
  ui->label_state_msg->setText(data.message.isEmpty() ? "-" : data.message);

  if (data.mode == 0) ui->radioButton_manual->setChecked(true);      // 실제 모드를 라디오 버튼에 반영
  else ui->radioButton_auto->setChecked(true);
}

// 영상처리 결과 표시 (검출 O/X, 위치 offset)
void MainWindow::onVision(const VisionData & data)
{
  markReceived(SRC_VISION);
  ui->label_vision_detected->setText(data.detected ? QString("O  %1").arg(data.label) : "X");
  ui->label_vision_offset->setText(QString::number(data.offset, 'f', 3));
}

// 로그 창에 [시:분:초] 와 함께 한 줄 추가하고 맨 아래로 스크롤
void MainWindow::onLog(const QString & text)
{
  ui->plainTextEdit_log->appendPlainText(
    QString("[%1] %2").arg(QDateTime::currentDateTime().toString("hh:mm:ss"), text));
  ui->plainTextEdit_log->verticalScrollBar()->setValue(ui->plainTextEdit_log->verticalScrollBar()->maximum());
}

// [노드 찾기]: 실행 중인 노드 목록으로 콤보박스 갱신 (하나도 없으면 기본 목록)
void MainWindow::refreshNodes()
{
  const QString current = ui->comboBox_param_node->currentText();
  const QStringList names = qnode.nodeNames();
  ui->comboBox_param_node->clear();
  ui->comboBox_param_node->addItems(names.isEmpty() ? kDefaultParamNodes : names);
  ui->comboBox_param_node->setCurrentText(current);
  onLog(QString("실행 중인 노드 %1개").arg(names.size()));
}

// [불러오기]: 선택한 노드의 파라미터 전체 요청 → 결과는 onParametersLoaded 로 옴
void MainWindow::loadParameters()
{
  QString name = ui->comboBox_param_node->currentText().trimmed();
  if (name.isEmpty()) return;
  if (!name.startsWith('/')) name.prepend('/');                       // "vision_node" 로 입력해도 "/vision_node" 로 맞춤
  qnode.loadParameters(name);
}

// 받은 파라미터를 이름순으로 표에 채움. 이름/타입 칸은 수정 불가, 값 칸만 수정 가능
void MainWindow::onParametersLoaded(const QString & node_name, const QVector<ParamData> & params)
{
  param_node_ = node_name;
  QTableWidget * table = ui->tableWidget_params;
  table->blockSignals(true);                                          // 채우는 동안 itemChanged(노란색 표시)가 불리지 않게
  table->setRowCount(0);

  QVector<ParamData> sorted = params;
  std::sort(sorted.begin(), sorted.end(), [](const ParamData & a, const ParamData & b) { return a.name < b.name; });

  for (const ParamData & p : sorted) {
    const int row = table->rowCount();
    table->insertRow(row);

    auto * name_item = new QTableWidgetItem(p.name);
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    auto * type_item = new QTableWidgetItem(p.type_name);
    type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
    auto * value_item = new QTableWidgetItem(p.value);
    value_item->setData(kParamTypeRole, p.type);
    value_item->setData(kParamOriginalRole, p.value);

    table->setItem(row, 0, name_item);
    table->setItem(row, 1, type_item);
    table->setItem(row, 2, value_item);
  }
  table->blockSignals(false);
  onLog(QString("%1 파라미터 %2개 불러옴").arg(node_name).arg(sorted.size()));
}

// [적용]: 원래 값과 달라진 칸만 모아서 QNode 로 전달
void MainWindow::applyParameters()
{
  if (param_node_.isEmpty()) {
    onLog("먼저 [불러오기]를 누르세요");
    return;
  }
  QVector<ParamData> changed;
  QTableWidget * table = ui->tableWidget_params;
  for (int row = 0; row < table->rowCount(); row++) {
    const QTableWidgetItem * value_item = table->item(row, 2);
    if (value_item->text() == value_item->data(kParamOriginalRole).toString()) continue;
    changed.push_back({table->item(row, 0)->text(), value_item->data(kParamTypeRole).toInt(),
                       table->item(row, 1)->text(), value_item->text()});
  }
  if (changed.isEmpty()) {
    onLog("바뀐 값이 없습니다");
    return;
  }
  qnode.setParameters(param_node_, changed);
}
