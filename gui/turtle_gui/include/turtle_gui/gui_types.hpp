#ifndef TURTLE_GUI_GUI_TYPES_HPP
#define TURTLE_GUI_GUI_TYPES_HPP

#include <QMetaType>
#include <QString>
#include <QVector>

// QNode -> MainWindow 로 넘기는 데이터 (ROS 타입을 GUI 쪽에 노출하지 않기 위함)

enum ImageSource { IMAGE_CAMERA = 0, IMAGE_VISION = 1 };

struct PsdData
{
  int raw[3];          // 좌, 중앙, 우
  int mm[3];
};

struct MotorData
{
  double left_mps;
  double right_mps;
  int left_error;
  int right_error;
};

struct RobotStateData
{
  int mode;            // 0 수동, 1 자율
  bool running;
  bool estop;
  bool stm32_connected;
  int mcu_error_flags;
  QString message;
};

struct VisionData
{
  bool detected;
  double offset;
  QString label;
};

struct ParamData
{
  QString name;
  int type;            // rclcpp::ParameterType 값
  QString type_name;
  QString value;       // 문자열로 표시/편집
};

Q_DECLARE_METATYPE(PsdData)
Q_DECLARE_METATYPE(MotorData)
Q_DECLARE_METATYPE(RobotStateData)
Q_DECLARE_METATYPE(VisionData)
Q_DECLARE_METATYPE(ParamData)
Q_DECLARE_METATYPE(QVector<ParamData>)

#endif
