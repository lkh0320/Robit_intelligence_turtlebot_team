#ifndef TURTLE_GUI_MAIN_WINDOW_HPP
#define TURTLE_GUI_MAIN_WINDOW_HPP

#include <QElapsedTimer>
#include <QLabel>
#include <QMainWindow>
#include <QPixmap>
#include <QTimer>

#include "turtle_gui/gui_types.hpp"
#include "turtle_gui/qnode.hpp"

namespace Ui
{
class MainWindowDesign;
}

// 화면 표시와 사용자 입력만 담당. ROS 통신은 전부 QNode 를 통해서만 한다
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow() override;

protected:
  bool eventFilter(QObject * watched, QEvent * event) override;
  void closeEvent(QCloseEvent * event) override;

private slots:
  void onImage(const QImage & image);
  void onPsd(const PsdData & data);
  void onMotor(const MotorData & data);
  void onRobotState(const RobotStateData & data);
  void onVision(const VisionData & data);
  void onParametersLoaded(const QString & node_name, const QVector<ParamData> & params);
  void onLog(const QString & text);
  void updateStatus();                                                // 연결 램프, fps 갱신 (0.5초)
  void sendManualCmd();                                               // 수동 주행 명령 발행 (0.1초)
  void loadParameters();
  void applyParameters();
  void refreshNodes();

private:
  enum Source { SRC_CAMERA, SRC_VISION, SRC_PSD, SRC_MOTOR, SRC_ROBOT, SRC_COUNT };
  enum DriveKey { KEY_W, KEY_A, KEY_S, KEY_D, KEY_COUNT };

  void markReceived(Source source);
  void setLamp(QLabel * lamp, int state);                             // 0 없음, 1 정상, 2 끊김
  void setDrive(DriveKey key, bool pressed);
  void clearDrive();
  double linearSpeed() const;
  double angularSpeed() const;

  Ui::MainWindowDesign * ui;
  QNode qnode;

  QTimer status_timer_;
  QTimer drive_timer_;
  QElapsedTimer last_rx_[SRC_COUNT];

  bool drive_[KEY_COUNT] = {false, false, false, false};
  bool was_driving_ = false;

  int frame_count_ = 0;
  QPixmap last_pixmap_;
  bool stm32_connected_ = false;
  QString param_node_;
};

#endif
