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
//
//  - 화면 배치(버튼, 라벨 위치)는 ui/main_window.ui 에 있음 → Qt Designer 로 수정
//  - 코드에서 위젯은 ui->위젯이름 으로 접근 (이름은 Qt Designer 의 objectName)
//  - on*() slot: QNode 가 데이터를 받으면 호출되어 화면을 갱신
//  - 버튼/키 입력 → qnode.callRun() 같은 함수 호출로 ROS 에 전달
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
  // QNode 에서 데이터가 오면 화면에 표시
  void onImage(const QImage & image);
  void onPsd(const PsdData & data);
  void onMotor(const MotorData & data);
  void onRobotState(const RobotStateData & data);
  void onVision(const VisionData & data);
  void onParametersLoaded(const QString & node_name, const QVector<ParamData> & params);
  void onLog(const QString & text);

  // 타이머, 버튼에서 호출
  void updateStatus();                                                // 연결 램프, fps 갱신 (0.5초)
  void sendManualCmd();                                               // 수동 주행 명령 발행 (0.1초)
  void loadParameters();
  void applyParameters();
  void refreshNodes();

private:
  enum Source { SRC_CAMERA, SRC_VISION, SRC_PSD, SRC_MOTOR, SRC_ROBOT, SRC_COUNT };  // 연결 램프 종류 (SRC_COUNT 는 개수)
  enum DriveKey { KEY_W, KEY_A, KEY_S, KEY_D, KEY_COUNT };           // 수동 주행 키

  void markReceived(Source source);                                   // "방금 받았음" 기록 → 램프 초록
  void setLamp(QLabel * lamp, int state);                             // 0 없음, 1 정상, 2 끊김
  void setDrive(DriveKey key, bool pressed);
  void clearDrive();
  double linearSpeed() const;                                        // 슬라이더 값 → m/s
  double angularSpeed() const;                                       // 슬라이더 값 → rad/s

  Ui::MainWindowDesign * ui;                                          // .ui 에서 자동 생성된 위젯 모음
  QNode qnode;                                                        // ROS 통신 담당 (생성 즉시 스레드 시작)

  QTimer status_timer_;
  QTimer drive_timer_;
  QElapsedTimer last_rx_[SRC_COUNT];                                  // 종류별 마지막 수신 후 경과 시간

  bool drive_[KEY_COUNT] = {false, false, false, false};              // 지금 눌려 있는 키
  bool was_driving_ = false;                                          // 직전에 주행 중이었는지 (키를 뗄 때 정지 명령 1번 보내려고)

  int frame_count_ = 0;                                               // fps 계산용 (0.5초 동안 받은 프레임 수)
  QPixmap last_pixmap_;
  bool stm32_connected_ = false;
  QString param_node_;                                                // 파라미터 탭에 지금 불러온 노드 이름
};

#endif
