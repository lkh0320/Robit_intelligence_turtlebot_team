#ifndef TURTLE_GUI_GUI_TYPES_HPP
#define TURTLE_GUI_GUI_TYPES_HPP

#include <QMetaType>
#include <QString>
#include <QVector>

// QNode -> MainWindow 로 넘기는 데이터 (ROS 타입을 GUI 쪽에 노출하지 않기 위함)
//
// 왜 따로 구조체를 만들까?
//  - MainWindow(화면 코드)가 ROS 메시지 타입을 몰라도 되게 하려고. msg 가 바뀌어도 qnode.cpp 한 곳만 고치면 됨
//  - Qt 의 signal/slot 으로 스레드 사이에 데이터를 넘기려면 Qt 가 아는 타입이어야 함 → 맨 아래 Q_DECLARE_METATYPE
//
// 새 데이터를 화면에 띄우고 싶으면: 여기에 구조체 추가 → 맨 아래 Q_DECLARE_METATYPE 추가
//  → QNode 생성자에서 qRegisterMetaType 추가 (gui/README.md "기능 추가 방법" 참고)

// 영상 선택 콤보박스의 순서와 같음 (0: 카메라 원본, 1: 영상처리 결과)
enum ImageSource { IMAGE_CAMERA = 0, IMAGE_VISION = 1 };

// /sensor/psd (turtle_interfaces/PsdArray)
struct PsdData
{
  int raw[3];          // 좌, 중앙, 우
  int mm[3];           // 거리 [mm], 순서는 raw 와 같음
};

// /motor/state (turtle_interfaces/MotorState)
struct MotorData
{
  double left_mps;
  double right_mps;
  int left_error;      // Dynamixel 에러 코드, 0 = 정상
  int right_error;
};

// /robot/state (turtle_interfaces/RobotState)
struct RobotStateData
{
  int mode;            // 0 수동, 1 자율
  bool running;
  bool estop;
  bool stm32_connected;
  int mcu_error_flags; // protocol/turtle_protocol.h 의 ERR_* 비트
  QString message;
};

// /vision/result (turtle_interfaces/VisionResult)
struct VisionData
{
  bool detected;
  double offset;       // -1.0(왼쪽) ~ 1.0(오른쪽)
  QString label;
};

// 파라미터 탭 표의 한 줄 (노드 파라미터 하나)
struct ParamData
{
  QString name;
  int type;            // rclcpp::ParameterType 값
  QString type_name;
  QString value;       // 문자열로 표시/편집
};

// Qt 에게 위 구조체들을 signal/slot 인자로 쓸 수 있다고 알림
Q_DECLARE_METATYPE(PsdData)
Q_DECLARE_METATYPE(MotorData)
Q_DECLARE_METATYPE(RobotStateData)
Q_DECLARE_METATYPE(VisionData)
Q_DECLARE_METATYPE(ParamData)
Q_DECLARE_METATYPE(QVector<ParamData>)

#endif
