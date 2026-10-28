// 노드 파라미터 실시간 조절 패널
//   크롬 탭처럼 실행 중인 노드마다 탭 하나. 탭을 고르면 그 노드의 파라미터 목록이 나오고,
//   값을 바꾸면 바로 set_parameters 서비스로 보낸다 (ros2 param set 과 같음).
//   - 노드 목록은 2초마다 다시 찾는다. 꺼진 노드 탭은 회색 점, 다시 켜지면 값을 새로 읽는다
//   - 다른 곳(ros2 param set, 다른 GUI)에서 바꾼 값도 /parameter_events 로 받아 바로 반영
//   - 처음 읽은 값과 다른 파라미터는 주황색. "YAML 복사" 로 바뀐 값만 yaml 형식으로 클립보드에 복사
//   - "YAML 저장": 바뀐 값을 param_saver/save 로 보내면 Jetson 의 param_saver (task_planner 패키지) 가
//     src/*/config/<노드>.yaml 에 써 넣는다. 저장되면 그 값이 새 기준 (주황 해제)
//   탭 앞 점 색: 초록 = 노드가 바뀐 값을 바로 씀 (live_params 파라미터가 true),
//                회색 = 노드가 시작할 때만 읽음 (바꿔도 노드를 다시 켜야 반영), 빨강 = 노드 꺼짐
// 서비스 응답 콜백도 MainWindow 의 spin_some 안(GUI 스레드)에서 불리므로 위젯을 바로 건드려도 된다.
#ifndef TURTLE_GUI__PARAM_PANEL_HPP_
#define TURTLE_GUI__PARAM_PANEL_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <QWidget>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class QLabel;
class QLineEdit;
class QPushButton;
class QTabWidget;
class QTimer;
class QTreeWidget;
class QTreeWidgetItem;

class ParamPanel : public QWidget
{
  Q_OBJECT

public:
  explicit ParamPanel(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);

signals:
  void message(const QString & text);   // 로그창에 남길 글

private:
  // 파라미터 한 줄
  struct Row
  {
    QTreeWidgetItem * item = nullptr;
    QWidget * editor = nullptr;     // 타입에 맞는 입력칸 (체크박스 / 숫자칸 / 글자칸)
    rclcpp::Parameter initial;      // 처음 읽은 값 (주황 표시 / 원래대로 / YAML 기준)
    rclcpp::Parameter current;      // 노드에 지금 들어 있는 값
  };

  // 노드 탭 하나
  struct Tab
  {
    std::string node;               // 전체 이름 (/grid_drive)
    QWidget * page = nullptr;
    QTreeWidget * tree = nullptr;
    QLineEdit * filter = nullptr;
    QLabel * banner = nullptr;      // 위쪽 안내 (바로 반영 / 재시작 필요 / 꺼짐)
    QLabel * changed = nullptr;     // 아래 "바뀐 값 n개"
    std::shared_ptr<rclcpp::AsyncParametersClient> client;
    std::map<std::string, Row> rows;
    bool alive = true;              // 노드 목록에 있는지
    bool live = false;              // 노드가 live_params = true 를 가짐
    int generation = 0;             // 다시 읽을 때마다 +1 (늦게 온 옛 응답 무시)
    bool loading = false;           // 읽는 중 (2초마다 다시 찾을 때 겹쳐 읽지 않게)
  };

  void scanNodes();
  Tab & addTab(const std::string & node);
  // keep_initial: 처음 값을 유지 (새로고침). false = 노드가 다시 켜짐 -> 지금 값을 새 기준으로
  void load(Tab & tab, bool keep_initial);
  void build(Tab & tab, const std::vector<rclcpp::Parameter> & values,
    const std::vector<rcl_interfaces::msg::ParameterDescriptor> & descs, bool keep_initial);
  QWidget * makeEditor(Tab & tab, const rclcpp::Parameter & p,
    const rcl_interfaces::msg::ParameterDescriptor & d);
  void setEditor(Row & row, const rclcpp::Parameter & p);
  void send(Tab & tab, const rclcpp::Parameter & p);
  void onEvent(const rcl_interfaces::msg::ParameterEvent & ev);
  void applyFilter(Tab & tab);
  void refreshTab(Tab & tab);       // 탭 점 색 / 안내 / 바뀐 개수 / 줄 색
  QString yamlText(Tab & tab, int & count);   // 바뀐 값만 노드 yaml 형식으로
  void copyYaml(Tab & tab);
  void saveYaml(Tab & tab);         // param_saver (Jetson) 에 보내 config yaml 에 저장
  void onSaveResult(const std::string & json);
  void resetAll(Tab & tab);
  Tab * find(const std::string & node);

  rclcpp::Node::SharedPtr node_;
  QTabWidget * tabs_;
  QTimer * scan_timer_;
  bool user_picked_ = false;   // 사용자가 탭을 직접 고른 적 있음 (그 전에는 늘 맨 앞 탭을 보여줌)
  std::vector<std::unique_ptr<Tab>> tab_list_;   // 탭 순서와 무관한 소유 목록 (주소 고정)
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr event_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr save_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr save_sub_;
};

#endif  // TURTLE_GUI__PARAM_PANEL_HPP_
