// ParamPanel 구현: 노드 탭, 파라미터 목록 / 입력칸, set_parameters 송신, /parameter_events 반영
#include "turtle_gui/param_panel.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

#include <QApplication>
#include <QCheckBox>
#include <QClipboard>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTabBar>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWheelEvent>

namespace
{
// 탭 순서: 자주 만지는 노드를 앞에. 나머지는 이름 순
const std::vector<std::string> kOrder = {
  "grid_drive", "task_planner", "parking", "sign_detection", "stm_bridge", "bird_eye_view"};

// 목록에 안 보일 파라미터 (ROS 기본 / GUI 표시용)
bool hidden(const std::string & name)
{
  return name == "use_sim_time" || name == "live_params" ||
         name == "start_type_description_service" || name.rfind("qos_overrides.", 0) == 0;
}

// 탭으로 안 만들 노드 (GUI 자신 / ros2 cli 데몬 등)
bool skipNode(const std::string & name)
{
  for (const char * p : {"turtle_gui", "param_saver", "_ros2cli", "launch_ros", "transform_listener_impl"}) {
    if (name.rfind(p, 0) == 0) {
      return true;
    }
  }
  return false;
}

std::string baseName(const std::string & full)
{
  const auto pos = full.rfind('/');
  return pos == std::string::npos ? full : full.substr(pos + 1);
}

int rank(const std::string & full)
{
  const auto it = std::find(kOrder.begin(), kOrder.end(), baseName(full));
  return static_cast<int>(it - kOrder.begin());   // 목록에 없으면 kOrder.size()
}

// 탭 앞 점 (크롬 파비콘 자리)
QIcon dot(const QColor & color)
{
  QPixmap pm(12, 12);
  pm.fill(Qt::transparent);
  QPainter p(&pm);
  p.setRenderHint(QPainter::Antialiasing);
  p.setPen(Qt::NoPen);
  p.setBrush(color);
  p.drawEllipse(2, 2, 8, 8);
  return QIcon(pm);
}

// 마우스 휠은 칸을 클릭해 포커스가 있을 때만 값 변경 (목록 스크롤하다 값이 바뀌는 사고 방지)
class WheelGuardSpin : public QSpinBox
{
protected:
  void wheelEvent(QWheelEvent * e) override
  {
    if (hasFocus()) {
      QSpinBox::wheelEvent(e);
    } else {
      e->ignore();   // 부모(목록)로 넘겨 스크롤
    }
  }
};

class WheelGuardDoubleSpin : public QDoubleSpinBox
{
public:
  // 0.408000 대신 0.408 처럼 뒤쪽 0 을 지워 보여준다
  QString textFromValue(double v) const override
  {
    QString s = QString::number(v, 'f', decimals());
    if (s.contains('.')) {
      while (s.endsWith('0')) {s.chop(1);}
      if (s.endsWith('.')) {s.chop(1);}
    }
    return s;
  }

protected:
  void wheelEvent(QWheelEvent * e) override
  {
    if (hasFocus()) {
      QDoubleSpinBox::wheelEvent(e);
    } else {
      e->ignore();
    }
  }
};

// yaml 한 값 (실수는 정수처럼 보여도 소수점을 붙여야 다시 읽을 때 double 로 선언됨)
QString yamlValue(const rclcpp::Parameter & p)
{
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return p.as_bool() ? "true" : "false";
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return QString::number(p.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE: {
        QString s = QString::number(p.as_double(), 'g', 12);
        if (!s.contains('.') && !s.contains('e') && !s.contains("inf") && !s.contains("nan")) {
          s += ".0";
        }
        return s;
      }
    case rclcpp::ParameterType::PARAMETER_STRING:
      return "\"" + QString::fromStdString(p.as_string()).replace("\"", "\\\"") + "\"";
    default:
      return QString::fromStdString(p.value_to_string());
  }
}

QString typeName(uint8_t t)
{
  switch (t) {
    case rclcpp::ParameterType::PARAMETER_BOOL: return "bool";
    case rclcpp::ParameterType::PARAMETER_INTEGER: return "int";
    case rclcpp::ParameterType::PARAMETER_DOUBLE: return "double";
    case rclcpp::ParameterType::PARAMETER_STRING: return "string";
    default: return "배열";
  }
}
}  // namespace

ParamPanel::ParamPanel(rclcpp::Node::SharedPtr node, QWidget * parent)
: QWidget(parent), node_(std::move(node))
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  // 크롬 탭 모양: 둥근 윗모서리, 고른 탭은 아래 내용과 이어지는 흰색
  tabs_ = new QTabWidget;
  tabs_->setDocumentMode(true);
  tabs_->setMovable(true);
  tabs_->setUsesScrollButtons(true);
  tabs_->setElideMode(Qt::ElideRight);
  tabs_->tabBar()->setDrawBase(false);
  tabs_->tabBar()->setExpanding(false);
  tabs_->setStyleSheet(
    "QTabWidget::pane { background:#ffffff; border:1px solid #dde1e6; border-top:none;"
    "  border-bottom-left-radius:8px; border-bottom-right-radius:8px; }"
    "QTabBar { background:transparent; }"
    "QTabBar::tab { background:#e5e7eb; color:#4b5563; border:1px solid #dde1e6; border-bottom:none;"
    "  border-top-left-radius:9px; border-top-right-radius:9px; padding:6px 14px 6px 10px;"
    "  margin-right:2px; min-width:70px; }"
    "QTabBar::tab:selected { background:#ffffff; color:#111827; font-weight:bold; }"
    "QTabBar::tab:!selected:hover { background:#f3f4f6; }");

  // 탭 줄 오른쪽 끝: 노드 다시 찾기
  auto * rescan = new QToolButton;
  rescan->setText("⟳");
  rescan->setToolTip("노드 다시 찾기 / 지금 탭 값 다시 읽기");
  rescan->setFocusPolicy(Qt::NoFocus);
  rescan->setAutoRaise(true);
  connect(rescan, &QToolButton::clicked, this, [this] {
    scanNodes();
    if (auto * page = tabs_->currentWidget()) {
      for (auto & t : tab_list_) {
        if (t->page == page && t->alive) {
          load(*t, true);
        }
      }
    }
  });
  tabs_->setCornerWidget(rescan, Qt::TopRightCorner);
  connect(tabs_->tabBar(), &QTabBar::tabBarClicked, this, [this] {user_picked_ = true;});

  auto * title = new QLabel("파라미터");
  title->setStyleSheet("color:#6b7280; font-size:11px; font-weight:bold; padding:0 0 4px 2px;");
  layout->addWidget(title);
  layout->addWidget(tabs_, 1);

  // 다른 곳에서 바뀐 값도 반영 (우리 set 결과 확인 겸)
  event_sub_ = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", rclcpp::ParameterEventsQoS(),
    [this](rcl_interfaces::msg::ParameterEvent::ConstSharedPtr ev) {onEvent(*ev);});

  save_pub_ = node_->create_publisher<std_msgs::msg::String>("param_saver/save", 10);
  save_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "param_saver/result", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {onSaveResult(m->data);});

  scan_timer_ = new QTimer(this);
  connect(scan_timer_, &QTimer::timeout, this, &ParamPanel::scanNodes);
  scan_timer_->start(2000);
  QTimer::singleShot(300, this, &ParamPanel::scanNodes);   // 노드 발견 시간을 조금 기다린 뒤 첫 검색
}

ParamPanel::Tab * ParamPanel::find(const std::string & node)
{
  for (auto & t : tab_list_) {
    if (t->node == node) {
      return t.get();
    }
  }
  return nullptr;
}

// 실행 중인 노드 목록과 탭 맞추기: 새 노드 = 탭 추가, 사라짐 = 빨간 점, 다시 켜짐 = 새로 읽기
void ParamPanel::scanNodes()
{
  std::set<std::string> now;
  const std::string self = node_->get_fully_qualified_name();
  for (const auto & [name, ns] : node_->get_node_graph_interface()->get_node_names_and_namespaces()) {
    const std::string full = (ns.empty() || ns == "/") ? "/" + name : ns + "/" + name;
    if (full != self && !skipNode(name)) {
      now.insert(full);
    }
  }
  for (const auto & full : now) {
    Tab * t = find(full);
    if (!t) {
      t = &addTab(full);
      emit message(QString("파라미터: 노드 발견 %1").arg(QString::fromStdString(full)));
      load(*t, false);
    } else if (!t->alive) {
      t->alive = true;
      emit message(QString("파라미터: %1 다시 켜짐 -> 값 다시 읽음").arg(QString::fromStdString(full)));
      load(*t, false);
    } else if (t->rows.empty() && !t->loading) {
      load(*t, false);   // 아직 서비스가 준비 안 됐던 노드: 다시 시도
    }
  }
  for (auto & t : tab_list_) {
    if (t->alive && !now.count(t->node)) {
      t->alive = false;
      t->loading = false;
      refreshTab(*t);
    }
  }
}

ParamPanel::Tab & ParamPanel::addTab(const std::string & node)
{
  auto tab = std::make_unique<Tab>();
  Tab & t = *tab;
  t.node = node;
  t.client = std::make_shared<rclcpp::AsyncParametersClient>(node_, node);

  t.page = new QWidget;
  auto * v = new QVBoxLayout(t.page);
  v->setContentsMargins(8, 8, 8, 8);
  v->setSpacing(6);

  t.banner = new QLabel;
  t.banner->setWordWrap(true);
  v->addWidget(t.banner);

  t.filter = new QLineEdit;
  t.filter->setPlaceholderText("🔍  파라미터 검색 (예: margin, planner.)");
  t.filter->setClearButtonEnabled(true);
  t.filter->setStyleSheet("QLineEdit { border:1px solid #d1d5db; border-radius:6px; padding:4px 6px; }");
  connect(t.filter, &QLineEdit::textChanged, this, [this, &t] {applyFilter(t);});
  v->addWidget(t.filter);

  t.tree = new QTreeWidget;
  t.tree->setColumnCount(2);
  t.tree->setHeaderHidden(true);
  t.tree->setRootIsDecorated(true);
  t.tree->setIndentation(14);
  t.tree->setUniformRowHeights(true);
  t.tree->setSelectionMode(QAbstractItemView::NoSelection);
  t.tree->setFocusPolicy(Qt::NoFocus);
  t.tree->header()->setStretchLastSection(true);
  t.tree->header()->setSectionResizeMode(0, QHeaderView::Interactive);
  t.tree->setColumnWidth(0, 210);
  t.tree->setStyleSheet("QTreeWidget { border:none; } QTreeWidget::item { padding:1px 0; }");
  t.tree->addTopLevelItem(new QTreeWidgetItem(QStringList{"읽는 중..."}));
  v->addWidget(t.tree, 1);

  // 아래 줄: 바뀐 개수 / YAML 복사 / 원래대로
  auto * bottom = new QHBoxLayout;
  t.changed = new QLabel;
  t.changed->setStyleSheet("color:#6b7280;");
  bottom->addWidget(t.changed, 1);
  auto make_btn = [bottom](const QString & text, const QString & tip) {
      auto * b = new QPushButton(text);
      b->setFocusPolicy(Qt::NoFocus);
      b->setToolTip(tip);
      b->setStyleSheet("QPushButton { border:1px solid #d1d5db; border-radius:6px; padding:4px 10px;"
        " background:#f9fafb; } QPushButton:hover { background:#eef2f7; }");
      bottom->addWidget(b);
      return b;
    };
  auto * yaml = make_btn("YAML 복사", "처음 값과 다른 파라미터만 yaml 형식으로 클립보드에 복사 (config yaml 에 붙여 넣어 저장)");
  connect(yaml, &QPushButton::clicked, this, [this, &t] {copyYaml(t);});
  auto * save = make_btn("YAML 저장", "바뀐 값을 로봇(Jetson) 의 config yaml 에 저장 (주석 유지, 다음 실행부터 그 값으로 시작)");
  save->setStyleSheet(save->styleSheet().replace("#f9fafb", "#e8f5e9"));
  connect(save, &QPushButton::clicked, this, [this, &t] {saveYaml(t);});
  auto * reset = make_btn("원래대로", "바꾼 파라미터를 모두 처음 읽은 값으로 되돌림");
  connect(reset, &QPushButton::clicked, this, [this, &t] {resetAll(t);});
  v->addLayout(bottom);

  // 우선순위 순서 자리에 끼워 넣기
  int index = tabs_->count();
  for (int i = 0; i < tabs_->count(); ++i) {
    for (auto & other : tab_list_) {
      if (other->page == tabs_->widget(i)) {
        const int a = rank(node), b = rank(other->node);
        if (a < b || (a == b && node < other->node)) {
          index = std::min(index, i);
        }
      }
    }
  }
  tabs_->insertTab(index, t.page, QString::fromStdString(baseName(node)));
  tabs_->setTabToolTip(index, QString::fromStdString(node));
  if (!user_picked_) {
    tabs_->setCurrentIndex(0);   // 노드가 발견된 순서와 상관없이 처음엔 grid_drive 같은 앞 탭
  }
  tab_list_.push_back(std::move(tab));
  refreshTab(t);
  return t;
}

// 목록 -> 값 -> 설명(타입 범위, 읽기 전용) 순서로 비동기 요청. 응답은 spin_some 안에서 온다
void ParamPanel::load(Tab & tab, bool keep_initial)
{
  if (!tab.client->service_is_ready()) {
    return;   // 다음 scanNodes 때 다시 시도
  }
  tab.loading = true;
  const int gen = ++tab.generation;
  const std::string node = tab.node;
  auto fail = [this, node, gen](const char * what) {
      Tab * t = find(node);
      if (t && t->generation == gen) {
        t->loading = false;
        emit message(QString("파라미터 읽기 실패 (%1): %2").arg(QString::fromStdString(node), what));
      }
    };
  tab.client->list_parameters({}, 0,
    [this, node, gen, keep_initial, fail](
      std::shared_future<rcl_interfaces::msg::ListParametersResult> f) {
      Tab * t = find(node);
      if (!t || t->generation != gen) {return;}
      std::vector<std::string> names;
      try {
        names = f.get().names;
      } catch (const std::exception & e) {
        fail(e.what());
        return;
      }
      std::sort(names.begin(), names.end());
      t->client->get_parameters(names,
        [this, node, gen, names, keep_initial, fail](std::shared_future<std::vector<rclcpp::Parameter>> fv) {
          Tab * t = find(node);
          if (!t || t->generation != gen) {return;}
          std::vector<rclcpp::Parameter> values;
          try {
            values = fv.get();
          } catch (const std::exception & e) {
            fail(e.what());
            return;
          }
          t->client->describe_parameters(names,
            [this, node, gen, values, keep_initial, fail](
              std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> fd) {
              Tab * t = find(node);
              if (!t || t->generation != gen) {return;}
              try {
                build(*t, values, fd.get(), keep_initial);
              } catch (const std::exception & e) {
                fail(e.what());
              }
            });
        });
    });
}

// 받은 값으로 목록을 새로 만든다. "planner.linear" 처럼 점이 있으면 앞부분(planner)으로 묶음
void ParamPanel::build(
  Tab & tab, const std::vector<rclcpp::Parameter> & values,
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & descs, bool keep_initial)
{
  std::map<std::string, rclcpp::Parameter> old_initial;
  if (keep_initial) {
    for (const auto & [name, row] : tab.rows) {
      old_initial[name] = row.initial;
    }
  }
  const int scroll = tab.tree->verticalScrollBar() ? tab.tree->verticalScrollBar()->value() : 0;
  tab.rows.clear();
  tab.tree->clear();
  tab.live = false;

  std::map<std::string, QTreeWidgetItem *> groups;
  for (size_t i = 0; i < values.size(); ++i) {
    const rclcpp::Parameter & p = values[i];
    const std::string & name = p.get_name();
    if (name == "live_params") {
      tab.live = p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL && p.as_bool();
    }
    if (hidden(name) || p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      continue;
    }
    const rcl_interfaces::msg::ParameterDescriptor d =
      i < descs.size() ? descs[i] : rcl_interfaces::msg::ParameterDescriptor();

    QTreeWidgetItem * parent = nullptr;
    QString label = QString::fromStdString(name);
    const auto dot_pos = name.find('.');
    if (dot_pos != std::string::npos) {
      const std::string group = name.substr(0, dot_pos);
      auto & g = groups[group];
      if (!g) {
        g = new QTreeWidgetItem(QStringList{QString::fromStdString(group)});
        QFont f = g->font(0);
        f.setBold(true);
        g->setFont(0, f);
        g->setFirstColumnSpanned(true);
        g->setForeground(0, QColor("#374151"));
      }
      parent = g;
      label = QString::fromStdString(name.substr(dot_pos + 1));
    }
    auto * item = new QTreeWidgetItem(QStringList{label});
    QString tip = QString("<b>%1</b> (%2)").arg(QString::fromStdString(name), typeName(p.get_type()));
    if (!d.description.empty()) {
      tip += "<br>" + QString::fromStdString(d.description).toHtmlEscaped();
    }
    if (d.read_only) {
      tip += "<br><i>읽기 전용</i>";
    }
    item->setToolTip(0, tip);
    // 설명에 "재시작" 이 있으면 바로 반영이 안 되는 파라미터 (노드가 시작할 때만 씀)
    if (QString::fromStdString(d.description).contains("재시작")) {
      item->setText(0, label + "  ↻");
    }
    if (parent) {
      parent->addChild(item);
    } else {
      tab.tree->addTopLevelItem(item);
    }

    Row & row = tab.rows[name];
    row.item = item;
    row.current = p;
    const auto it = old_initial.find(name);
    row.initial = it != old_initial.end() ? it->second : p;
    row.editor = makeEditor(tab, p, d);
  }
  if (tab.rows.empty()) {
    tab.tree->addTopLevelItem(new QTreeWidgetItem(QStringList{"파라미터 없음"}));
  }
  // 묶음은 개별 파라미터 아래에 이름 순으로
  for (auto & [name, g] : groups) {
    tab.tree->addTopLevelItem(g);
  }
  // setItemWidget 은 아이템이 트리에 들어간 뒤에만 된다
  for (auto & [name, row] : tab.rows) {
    tab.tree->setItemWidget(row.item, 1, row.editor);
  }
  tab.tree->expandAll();
  // 이름 칸은 가장 긴 이름에 맞춤 (단, 목록 폭의 65% 까지)
  tab.tree->resizeColumnToContents(0);
  tab.tree->setColumnWidth(0, std::min(tab.tree->columnWidth(0) + 12,
    std::max(160, tab.tree->viewport()->width() * 65 / 100)));
  if (tab.tree->verticalScrollBar()) {
    tab.tree->verticalScrollBar()->setValue(scroll);
  }
  tab.loading = false;
  applyFilter(tab);
  refreshTab(tab);
}

QWidget * ParamPanel::makeEditor(
  Tab & tab, const rclcpp::Parameter & p, const rcl_interfaces::msg::ParameterDescriptor & d)
{
  const std::string name = p.get_name();
  const std::string node = tab.node;
  // 입력칸이 바뀐 값을 보낼 때 탭을 이름으로 다시 찾는다 (탭 / 줄이 그 사이 다시 만들어졌을 수 있음)
  auto sendValue = [this, node](const rclcpp::Parameter & np) {
      if (Tab * t = find(node)) {
        send(*t, np);
      }
    };
  QWidget * editor = nullptr;
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL: {
        auto * c = new QCheckBox;
        c->setChecked(p.as_bool());
        c->setFocusPolicy(Qt::NoFocus);
        connect(c, &QCheckBox::toggled, this, [sendValue, name](bool on) {
          sendValue(rclcpp::Parameter(name, on));
        });
        editor = c;
        break;
      }
    case rclcpp::ParameterType::PARAMETER_INTEGER: {
        auto * s = new WheelGuardSpin;
        s->setFocusPolicy(Qt::StrongFocus);
        if (!d.integer_range.empty()) {
          s->setRange(static_cast<int>(d.integer_range[0].from_value),
            static_cast<int>(d.integer_range[0].to_value));
          s->setSingleStep(std::max<int>(1, static_cast<int>(d.integer_range[0].step)));
        } else {
          s->setRange(-1000000000, 1000000000);
        }
        s->setValue(static_cast<int>(p.as_int()));
        s->setKeyboardTracking(false);   // 글자를 칠 때마다가 아니라 Enter / 칸을 벗어날 때 보냄
        connect(s, qOverload<int>(&QSpinBox::valueChanged), this, [sendValue, name](int v) {
          sendValue(rclcpp::Parameter(name, static_cast<int64_t>(v)));
        });
        editor = s;
        break;
      }
    case rclcpp::ParameterType::PARAMETER_DOUBLE: {
        auto * s = new WheelGuardDoubleSpin;
        s->setFocusPolicy(Qt::StrongFocus);
        s->setDecimals(6);
        const double v = p.as_double();
        if (!d.floating_point_range.empty()) {
          s->setRange(d.floating_point_range[0].from_value, d.floating_point_range[0].to_value);
        } else {
          s->setRange(-1e9, 1e9);
        }
        // 화살표 한 칸 = 값 자릿수보다 한 자리 아래 (0.408 -> 0.01, 150 -> 10, 0 -> 0.01)
        double step = v == 0.0 ? 0.01 : std::pow(10.0, std::floor(std::log10(std::fabs(v))) - 1);
        if (!d.floating_point_range.empty() && d.floating_point_range[0].step > 0) {
          step = d.floating_point_range[0].step;
        }
        s->setSingleStep(std::max(step, 1e-6));
        s->setValue(v);
        s->setKeyboardTracking(false);
        connect(s, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [sendValue, name](double nv) {
          sendValue(rclcpp::Parameter(name, nv));
        });
        editor = s;
        break;
      }
    case rclcpp::ParameterType::PARAMETER_STRING: {
        auto * e = new QLineEdit(QString::fromStdString(p.as_string()));
        connect(e, &QLineEdit::editingFinished, this, [this, e, node, name, sendValue] {
          Tab * t = find(node);
          if (!t || !t->rows.count(name)) {return;}
          const auto & cur = t->rows[name].current;
          if (cur.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
          cur.as_string() != e->text().toStdString())
          {
            sendValue(rclcpp::Parameter(name, e->text().toStdString()));
          }
        });
        editor = e;
        break;
      }
    default: {
        // 배열은 보기만 (바꿀 일이 거의 없고 입력 형식이 복잡함)
        auto * e = new QLineEdit(QString::fromStdString(p.value_to_string()));
        e->setReadOnly(true);
        e->setStyleSheet("color:#6b7280;");
        editor = e;
        break;
      }
  }
  if (d.read_only) {
    editor->setEnabled(false);
  }
  // 입력칸은 짧게 (오른쪽에 붙지 않고 이름 바로 옆)
  if (!qobject_cast<QCheckBox *>(editor)) {
    editor->setMaximumWidth(qobject_cast<QLineEdit *>(editor) ? 260 : 150);
  }
  return editor;
}

// 노드에 있는 값으로 입력칸을 맞춘다 (보내지 않도록 신호 막음)
void ParamPanel::setEditor(Row & row, const rclcpp::Parameter & p)
{
  const QSignalBlocker block(row.editor);
  if (auto * c = qobject_cast<QCheckBox *>(row.editor)) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {c->setChecked(p.as_bool());}
  } else if (auto * s = qobject_cast<QSpinBox *>(row.editor)) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {s->setValue(static_cast<int>(p.as_int()));}
  } else if (auto * ds = qobject_cast<QDoubleSpinBox *>(row.editor)) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {ds->setValue(p.as_double());}
  } else if (auto * e = qobject_cast<QLineEdit *>(row.editor)) {
    e->setText(p.get_type() == rclcpp::ParameterType::PARAMETER_STRING ?
      QString::fromStdString(p.as_string()) : QString::fromStdString(p.value_to_string()));
  }
}

// set_parameters 서비스로 하나 보냄. 거절되면 입력칸을 노드 값으로 되돌리고 이유를 로그에
void ParamPanel::send(Tab & tab, const rclcpp::Parameter & p)
{
  const std::string name = p.get_name();
  const std::string node = tab.node;
  const QString label = QString("%1  %2").arg(QString::fromStdString(baseName(node)),
      QString::fromStdString(name));
  auto it = tab.rows.find(name);
  if (it == tab.rows.end()) {
    return;
  }
  if (!tab.alive || !tab.client->service_is_ready()) {
    emit message(label + ": 노드가 꺼져 있어 못 보냄");
    setEditor(it->second, it->second.current);
    return;
  }
  const QString old_text = QString::fromStdString(it->second.current.value_to_string());
  tab.client->set_parameters({p},
    [this, node, name, p, label, old_text](
      std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
      Tab * t = find(node);
      if (!t) {return;}
      auto row = t->rows.find(name);
      if (row == t->rows.end()) {return;}
      QString reason;
      bool ok = false;
      try {
        const auto r = f.get();
        ok = !r.empty() && r[0].successful;
        if (!r.empty()) {reason = QString::fromStdString(r[0].reason);}
      } catch (const std::exception & e) {
        reason = e.what();
      }
      if (ok) {
        row->second.current = p;
        emit message(QString("%1: %2 → %3").arg(label, old_text,
          QString::fromStdString(p.value_to_string())));
      } else {
        emit message(QString("%1: 거절됨 (%2)").arg(label, reason.isEmpty() ? "이유 없음" : reason));
        setEditor(row->second, row->second.current);
      }
      refreshTab(*t);
    });
}

// /parameter_events: 노드 값이 바뀌면 (누가 바꿨든) 입력칸 갱신. 새로 선언된 파라미터가 있으면 다시 읽음
void ParamPanel::onEvent(const rcl_interfaces::msg::ParameterEvent & ev)
{
  Tab * t = find(ev.node);
  if (!t || t->loading) {
    return;
  }
  bool reload = false;
  for (const auto & msg : ev.changed_parameters) {
    auto row = t->rows.find(msg.name);
    if (row != t->rows.end()) {
      const auto p = rclcpp::Parameter::from_parameter_msg(msg);
      row->second.current = p;
      setEditor(row->second, p);
    }
  }
  for (const auto & msg : ev.new_parameters) {
    if (!hidden(msg.name) && !t->rows.count(msg.name)) {
      reload = true;
    }
  }
  if (reload) {
    load(*t, true);
  } else {
    refreshTab(*t);
  }
}

void ParamPanel::applyFilter(Tab & tab)
{
  const QString text = tab.filter->text().trimmed();
  for (auto & [name, row] : tab.rows) {
    row.item->setHidden(!text.isEmpty() && !QString::fromStdString(name).contains(text, Qt::CaseInsensitive));
  }
  // 묶음은 안에 보이는 것이 하나도 없으면 숨김
  for (int i = 0; i < tab.tree->topLevelItemCount(); ++i) {
    auto * top = tab.tree->topLevelItem(i);
    if (top->childCount() == 0) {continue;}
    bool any = false;
    for (int c = 0; c < top->childCount(); ++c) {
      any = any || !top->child(c)->isHidden();
    }
    top->setHidden(!any);
  }
}

void ParamPanel::refreshTab(Tab & tab)
{
  const int index = tabs_->indexOf(tab.page);
  if (!tab.alive) {
    tabs_->setTabIcon(index, dot(QColor("#d32f2f")));
    tab.banner->setText("<span style='color:#d32f2f'>● 노드 꺼짐</span> — 다시 켜지면 값을 새로 읽습니다");
  } else if (tab.live) {
    tabs_->setTabIcon(index, dot(QColor("#2e7d32")));
    tab.banner->setText("<span style='color:#2e7d32'>● 바꾸면 바로 반영</span>"
      "<span style='color:#6b7280'> — ↻ 표시는 재시작해야 반영</span>");
  } else {
    tabs_->setTabIcon(index, dot(QColor("#9ca3af")));
    tab.banner->setText("<span style='color:#b45309'>● 시작할 때만 읽는 노드</span>"
      "<span style='color:#6b7280'> — 바꾼 값은 노드를 다시 켜야 반영 (YAML 복사로 저장)</span>");
  }
  int changed = 0;
  for (auto & [name, row] : tab.rows) {
    const bool diff = row.current.get_parameter_value() != row.initial.get_parameter_value();
    changed += diff;
    QFont f = row.item->font(0);
    f.setBold(diff);
    row.item->setFont(0, f);
    row.item->setForeground(0, diff ? QColor("#c2410c") : QColor("#111827"));
    row.item->setToolTip(1, diff ? "처음 값: " + QString::fromStdString(row.initial.value_to_string()) : "");
  }
  tab.changed->setText(changed ? QString("<span style='color:#c2410c'>바뀐 값 %1개</span>").arg(changed) :
    QString("파라미터 %1개").arg(tab.rows.size()));
}

// 처음 값과 다른 것만 노드 yaml 형식으로 (점 이름은 들여쓰기로 풀어서)
QString ParamPanel::yamlText(Tab & tab, int & count)
{
  std::vector<const Row *> rows;
  for (auto & [name, row] : tab.rows) {
    if (row.current.get_parameter_value() != row.initial.get_parameter_value()) {
      rows.push_back(&row);
    }
  }
  count = static_cast<int>(rows.size());
  if (rows.empty()) {
    return "";
  }
  QString out = QString::fromStdString(baseName(tab.node)) + ":\n  ros__parameters:\n";
  QStringList prev;   // 직전 줄의 묶음 경로
  for (const Row * row : rows) {   // rows 는 이름 순 (std::map)
    QStringList parts = QString::fromStdString(row->current.get_name()).split('.');
    const QString leaf = parts.takeLast();
    int same = 0;
    while (same < parts.size() && same < prev.size() && parts[same] == prev[same]) {++same;}
    for (int i = same; i < parts.size(); ++i) {
      out += QString(4 + 2 * i, ' ') + parts[i] + ":\n";
    }
    out += QString(4 + 2 * parts.size(), ' ') + leaf + ": " + yamlValue(row->current) + "\n";
    prev = parts;
  }
  return out;
}

void ParamPanel::copyYaml(Tab & tab)
{
  int n = 0;
  const QString out = yamlText(tab, n);
  if (n == 0) {
    emit message(QString("%1: 바뀐 값 없음").arg(QString::fromStdString(baseName(tab.node))));
    return;
  }
  QApplication::clipboard()->setText(out);
  emit message(QString("%1: 바뀐 값 %2개 YAML 클립보드에 복사\n%3")
    .arg(QString::fromStdString(baseName(tab.node))).arg(n).arg(out.trimmed()));
}

void ParamPanel::saveYaml(Tab & tab)
{
  const QString name = QString::fromStdString(baseName(tab.node));
  int n = 0;
  const QString out = yamlText(tab, n);
  if (n == 0) {
    emit message(name + ": 저장할 바뀐 값 없음");
    return;
  }
  if (save_pub_->get_subscription_count() == 0) {
    emit message(name + ": param_saver 가 안 떠 있어 저장 못 함 (Jetson grid_drive_bringup 에서 켜짐). "
      "YAML 복사로 직접 붙여 넣기");
    return;
  }
  std_msgs::msg::String msg;
  msg.data = out.toStdString();
  save_pub_->publish(msg);
  emit message(QString("%1: 바뀐 값 %2개 저장 요청").arg(name).arg(n));
}

// param_saver 응답 {"node", "ok", "names", "msg"}: 저장된 값은 새 기준으로 (주황 해제)
void ParamPanel::onSaveResult(const std::string & json)
{
  const QJsonObject r = QJsonDocument::fromJson(QByteArray::fromStdString(json)).object();
  const QString node = r.value("node").toString();
  if (!r.value("ok").toBool()) {
    emit message(QString("%1: YAML 저장 실패 - %2").arg(node, r.value("msg").toString()));
    return;
  }
  for (auto & t : tab_list_) {
    if (QString::fromStdString(baseName(t->node)) != node) {continue;}
    for (const auto & v : r.value("names").toArray()) {
      auto row = t->rows.find(v.toString().toStdString());
      if (row != t->rows.end()) {
        row->second.initial = row->second.current;
      }
    }
    refreshTab(*t);
  }
  emit message(QString("%1: YAML 저장 완료 - %2").arg(node, r.value("msg").toString()));
}

void ParamPanel::resetAll(Tab & tab)
{
  int n = 0;
  for (auto & [name, row] : tab.rows) {
    if (row.current.get_parameter_value() != row.initial.get_parameter_value()) {
      setEditor(row, row.initial);
      send(tab, row.initial);
      ++n;
    }
  }
  if (n == 0) {
    emit message(QString("%1: 되돌릴 값 없음").arg(QString::fromStdString(baseName(tab.node))));
  }
}
