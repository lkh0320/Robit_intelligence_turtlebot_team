#include "turtle_gui/main_window.hpp"

#include <algorithm>

#include <QCloseEvent>
#include <QDateTime>
#include <QHeaderView>
#include <QKeyEvent>
#include <QScrollBar>

#include "ui_main_window.h"

namespace
{
const QStringList kDefaultParamNodes = {"/camera_node", "/vision_node", "/control_node", "/stm32_bridge_node"};
const int kParamTypeRole = Qt::UserRole;
const int kParamOriginalRole = Qt::UserRole + 1;
}  // namespace

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);
  ui->plainTextEdit_log->setMaximumBlockCount(500);

  ui->comboBox_param_node->addItems(kDefaultParamNodes);
  ui->tableWidget_params->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  ui->tableWidget_params->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  ui->tableWidget_params->horizontalHeader()->setStretchLastSection(true);
  ui->tableWidget_params->verticalHeader()->setVisible(false);

  for (QLabel * lamp : {ui->label_lamp_camera, ui->label_lamp_vision, ui->label_lamp_psd,
                        ui->label_lamp_motor, ui->label_lamp_robot, ui->label_lamp_stm32}) {
    lamp->setMinimumHeight(28);
    setLamp(lamp, 0);
  }

  // QNode -> 화면
  connect(&qnode, &QNode::imageReceived, this, &MainWindow::onImage);
  connect(&qnode, &QNode::psdReceived, this, &MainWindow::onPsd);
  connect(&qnode, &QNode::motorReceived, this, &MainWindow::onMotor);
  connect(&qnode, &QNode::robotStateReceived, this, &MainWindow::onRobotState);
  connect(&qnode, &QNode::visionReceived, this, &MainWindow::onVision);
  connect(&qnode, &QNode::parametersLoaded, this, &MainWindow::onParametersLoaded);
  connect(&qnode, &QNode::parametersApplied, this, [this](const QString & node_name) {
    qnode.loadParameters(node_name);
  });
  connect(&qnode, &QNode::logMessage, this, &MainWindow::onLog);
  connect(&qnode, &QNode::rosShutdown, this, &MainWindow::close);

  // 영상 선택
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
    clearDrive();
    qnode.callEstop(true);
  });
  connect(ui->pushButton_estop_release, &QPushButton::clicked, this, [this]() { qnode.callEstop(false); });
  connect(ui->radioButton_manual, &QRadioButton::clicked, this, [this]() { qnode.callSetMode(0); });
  connect(ui->radioButton_auto, &QRadioButton::clicked, this, [this]() { qnode.callSetMode(1); });

  // 수동 주행 버튼 (누르는 동안만 주행)
  const QList<QPair<QPushButton *, DriveKey>> drive_buttons = {
    {ui->pushButton_w, KEY_W}, {ui->pushButton_a, KEY_A}, {ui->pushButton_s, KEY_S}, {ui->pushButton_d, KEY_D}};
  for (const auto & pair : drive_buttons) {
    const DriveKey key = pair.second;
    connect(pair.first, &QPushButton::pressed, this, [this, key]() { setDrive(key, true); });
    connect(pair.first, &QPushButton::released, this, [this, key]() { setDrive(key, false); });
  }
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
    if (item->column() != 2) return;
    const bool edited = item->text() != item->data(kParamOriginalRole).toString();
    item->setBackground(edited ? QColor("#fff59d") : QColor(Qt::transparent));  // 수정된 칸 노란색
  });
  connect(ui->tabWidget, &QTabWidget::currentChanged, this, [this]() { clearDrive(); });

  connect(&status_timer_, &QTimer::timeout, this, &MainWindow::updateStatus);
  connect(&drive_timer_, &QTimer::timeout, this, &MainWindow::sendManualCmd);
  status_timer_.start(500);
  drive_timer_.start(100);

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
  if (!key_event->isAutoRepeat()) setDrive(key, pressed);
  return true;
}

void MainWindow::setDrive(DriveKey key, bool pressed)
{
  drive_[key] = pressed;
  QPushButton * buttons[KEY_COUNT] = {ui->pushButton_w, ui->pushButton_a, ui->pushButton_s, ui->pushButton_d};
  buttons[key]->setDown(pressed);                                     // 키보드로 눌러도 버튼이 눌린 모양
}

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

void MainWindow::sendManualCmd()
{
  const int forward = (drive_[KEY_W] ? 1 : 0) - (drive_[KEY_S] ? 1 : 0);
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

void MainWindow::markReceived(Source source)
{
  last_rx_[source].restart();
}

void MainWindow::setLamp(QLabel * lamp, int state)
{
  static const char * colors[] = {"#9e9e9e", "#43a047", "#e53935"};
  lamp->setStyleSheet(QString("background-color: %1; color: white; border-radius: 4px; font-weight: bold;")
                        .arg(colors[state]));
}

void MainWindow::updateStatus()
{
  QLabel * lamps[SRC_COUNT] = {ui->label_lamp_camera, ui->label_lamp_vision, ui->label_lamp_psd,
                               ui->label_lamp_motor, ui->label_lamp_robot};
  for (int i = 0; i < SRC_COUNT; i++) {
    if (!last_rx_[i].isValid()) setLamp(lamps[i], 0);
    else setLamp(lamps[i], last_rx_[i].elapsed() < 1000 ? 1 : 2);
  }

  const bool robot_alive = last_rx_[SRC_ROBOT].isValid() && last_rx_[SRC_ROBOT].elapsed() < 1000;
  if (!last_rx_[SRC_ROBOT].isValid()) setLamp(ui->label_lamp_stm32, 0);
  else setLamp(ui->label_lamp_stm32, robot_alive && stm32_connected_ ? 1 : 2);

  ui->label_fps->setText(QString("%1 fps").arg(frame_count_ * 2));   // 0.5초마다 세므로 x2
  frame_count_ = 0;
}

void MainWindow::onImage(const QImage & image)
{
  markReceived(SRC_CAMERA);
  frame_count_++;
  last_pixmap_ = QPixmap::fromImage(image);
  ui->label_image->setPixmap(
    last_pixmap_.scaled(ui->label_image->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
}

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

void MainWindow::onVision(const VisionData & data)
{
  markReceived(SRC_VISION);
  ui->label_vision_detected->setText(data.detected ? QString("O  %1").arg(data.label) : "X");
  ui->label_vision_offset->setText(QString::number(data.offset, 'f', 3));
}

void MainWindow::onLog(const QString & text)
{
  ui->plainTextEdit_log->appendPlainText(
    QString("[%1] %2").arg(QDateTime::currentDateTime().toString("hh:mm:ss"), text));
  ui->plainTextEdit_log->verticalScrollBar()->setValue(ui->plainTextEdit_log->verticalScrollBar()->maximum());
}

void MainWindow::refreshNodes()
{
  const QString current = ui->comboBox_param_node->currentText();
  const QStringList names = qnode.nodeNames();
  ui->comboBox_param_node->clear();
  ui->comboBox_param_node->addItems(names.isEmpty() ? kDefaultParamNodes : names);
  ui->comboBox_param_node->setCurrentText(current);
  onLog(QString("실행 중인 노드 %1개").arg(names.size()));
}

void MainWindow::loadParameters()
{
  QString name = ui->comboBox_param_node->currentText().trimmed();
  if (name.isEmpty()) return;
  if (!name.startsWith('/')) name.prepend('/');
  qnode.loadParameters(name);
}

void MainWindow::onParametersLoaded(const QString & node_name, const QVector<ParamData> & params)
{
  param_node_ = node_name;
  QTableWidget * table = ui->tableWidget_params;
  table->blockSignals(true);
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
