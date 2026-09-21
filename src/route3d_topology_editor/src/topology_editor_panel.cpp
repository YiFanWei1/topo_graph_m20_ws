#include "route3d_topology_editor/topology_editor_panel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSpinBox>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QVariant>

#include <geometry_msgs/msg/point.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace route3d_topology_editor
{
namespace
{

using json = nlohmann::json;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

QDoubleSpinBox * realBox(
  const double minimum = -1000000.0, const double maximum = 1000000.0,
  const int decimals = 4)
{
  auto * box = new QDoubleSpinBox();
  box->setRange(minimum, maximum);
  box->setDecimals(decimals);
  box->setSingleStep(0.05);
  return box;
}

std_msgs::msg::ColorRGBA rgba(float red, float green, float blue, float alpha = 1.0F)
{
  std_msgs::msg::ColorRGBA result;
  result.r = red;
  result.g = green;
  result.b = blue;
  result.a = alpha;
  return result;
}

geometry_msgs::msg::Point point(const json & values, double z_offset = 0.0)
{
  geometry_msgs::msg::Point result;
  result.x = values.at(0).get<double>();
  result.y = values.at(1).get<double>();
  result.z = values.at(2).get<double>() + z_offset;
  return result;
}

Marker marker(
  const std::string & frame_id, const std::string & name_space, int id, int type,
  const rclcpp::Time & stamp)
{
  Marker result;
  result.header.frame_id = frame_id;
  result.header.stamp = stamp;
  result.ns = name_space;
  result.id = id;
  result.type = type;
  result.action = Marker::ADD;
  result.pose.orientation.w = 1.0;
  return result;
}

std_msgs::msg::ColorRGBA vertexColor(const json & vertex)
{
  const auto & meta = vertex.at("meta");
  if (meta.value("isJunction", false)) {
    return rgba(0.95F, 0.15F, 0.95F);
  }
  if (meta.value("isCorner", false)) {
    return rgba(1.0F, 0.15F, 0.12F);
  }
  if (meta.value("isSlope", false)) {
    return rgba(1.0F, 0.82F, 0.10F);
  }
  return rgba(0.10F, 0.55F, 1.0F);
}

std_msgs::msg::ColorRGBA edgeColor(int mode)
{
  switch (mode) {
    case 0: return rgba(0.20F, 0.55F, 1.0F);
    case 1: return rgba(0.10F, 0.90F, 0.25F);
    case 2: return rgba(1.0F, 0.20F, 0.15F);
    case 3: return rgba(0.65F, 0.65F, 0.65F);
    case 4: return rgba(0.65F, 0.20F, 1.0F);
    default: return rgba(1.0F, 1.0F, 1.0F);
  }
}

std::vector<int> sortedIds(const json & values)
{
  std::vector<int> result;
  result.reserve(values.size());
  for (const auto & [key, value] : values.items()) {
    (void)value;
    result.push_back(std::stoi(key));
  }
  std::sort(result.begin(), result.end());
  return result;
}

}  // namespace

TopologyEditorPanel::TopologyEditorPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
  setLoadedState(false);
}

void TopologyEditorPanel::buildUi()
{
  auto * root = new QVBoxLayout(this);
  auto * file_row = new QHBoxLayout();
  path_edit_ = new QLineEdit();
  path_edit_->setPlaceholderText("topoGraph_data.json 路径");
  auto * browse_button = new QPushButton("浏览");
  load_button_ = new QPushButton("加载");
  file_row->addWidget(path_edit_, 1);
  file_row->addWidget(browse_button);
  file_row->addWidget(load_button_);
  root->addLayout(file_row);

  auto * action_row = new QHBoxLayout();
  save_button_ = new QPushButton("保存");
  save_as_button_ = new QPushButton("另存为");
  undo_button_ = new QPushButton("撤销");
  redo_button_ = new QPushButton("重做");
  action_row->addWidget(save_button_);
  action_row->addWidget(save_as_button_);
  action_row->addWidget(undo_button_);
  action_row->addWidget(redo_button_);
  root->addLayout(action_row);

  auto * all_edges_row = new QHBoxLayout();
  all_stop_button_ = new QPushButton("全图停障（0 + PID）");
  all_avoid_button_ = new QPushButton("全图避障（1 + Efficient 3D）");
  all_edges_row->addWidget(all_stop_button_);
  all_edges_row->addWidget(all_avoid_button_);
  root->addLayout(all_edges_row);

  auto * graph_edit_row = new QHBoxLayout();
  add_vertex_mode_button_ = new QPushButton("添加点");
  add_edge_mode_button_ = new QPushButton("添加边（点选两点）");
  move_vertex_mode_button_ = new QPushButton("移动当前点");
  add_vertex_mode_button_->setCheckable(true);
  add_edge_mode_button_->setCheckable(true);
  move_vertex_mode_button_->setCheckable(true);
  graph_edit_row->addWidget(add_vertex_mode_button_);
  graph_edit_row->addWidget(add_edge_mode_button_);
  graph_edit_row->addWidget(move_vertex_mode_button_);
  root->addLayout(graph_edit_row);

  auto * delete_row = new QHBoxLayout();
  delete_vertex_button_ = new QPushButton("删除选中点及关联边");
  delete_edge_button_ = new QPushButton("删除选中边");
  delete_row->addWidget(delete_vertex_button_);
  delete_row->addWidget(delete_edge_button_);
  root->addLayout(delete_row);

  auto * selection_row = new QHBoxLayout();
  kind_combo_ = new QComboBox();
  kind_combo_->addItems({"顶点", "边"});
  id_combo_ = new QComboBox();
  selection_row->addWidget(kind_combo_);
  selection_row->addWidget(id_combo_, 1);
  root->addLayout(selection_row);

  auto * batch_row = new QHBoxLayout();
  batch_pick_enabled_ = new QCheckBox("批量点选起点/终点");
  batch_start_button_ = new QPushButton("当前点=起点");
  batch_end_button_ = new QPushButton("当前点=终点");
  batch_clear_button_ = new QPushButton("清除范围");
  batch_row->addWidget(batch_pick_enabled_);
  batch_row->addWidget(batch_start_button_);
  batch_row->addWidget(batch_end_button_);
  batch_row->addWidget(batch_clear_button_);
  root->addLayout(batch_row);
  batch_status_label_ = new QLabel("批量范围：未选择（起终点间最短拓扑路径）");
  batch_status_label_->setWordWrap(true);
  root->addWidget(batch_status_label_);

  auto * batch_field_row = new QHBoxLayout();
  batch_field_row->addWidget(new QLabel("批量字段"));
  batch_field_combo_ = new QComboBox();
  batch_field_row->addWidget(batch_field_combo_, 1);
  root->addLayout(batch_field_row);
  populateBatchFields(0);

  editor_stack_ = new QStackedWidget();
  editor_stack_->addWidget(buildVertexEditor());
  editor_stack_->addWidget(buildEdgeEditor());
  auto * scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setWidget(editor_stack_);
  root->addWidget(scroll, 1);

  auto * apply_button = new QPushButton("应用修改（尚未写入文件）");
  root->addWidget(apply_button);
  batch_apply_button_ = new QPushButton("批量应用所选顶点属性（不改坐标/RPY/PCD）");
  batch_apply_button_->setToolTip(
    "顶点页不批量修改 XYZ、RPY 和 PCD；边页修改路径内所有边属性");
  root->addWidget(batch_apply_button_);
  status_label_ = new QLabel("请选择拓扑文件");
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);

  connect(browse_button, &QPushButton::clicked, this, &TopologyEditorPanel::browseGraph);
  connect(load_button_, &QPushButton::clicked, this, &TopologyEditorPanel::loadGraph);
  connect(save_button_, &QPushButton::clicked, this, &TopologyEditorPanel::saveGraph);
  connect(save_as_button_, &QPushButton::clicked, this, &TopologyEditorPanel::saveGraphAs);
  connect(undo_button_, &QPushButton::clicked, this, &TopologyEditorPanel::undo);
  connect(redo_button_, &QPushButton::clicked, this, &TopologyEditorPanel::redo);
  connect(all_stop_button_, &QPushButton::clicked, this, &TopologyEditorPanel::applyAllStop);
  connect(all_avoid_button_, &QPushButton::clicked, this, &TopologyEditorPanel::applyAllAvoid);
  connect(
    add_vertex_mode_button_, &QPushButton::toggled,
    this, &TopologyEditorPanel::setAddVertexMode);
  connect(
    add_edge_mode_button_, &QPushButton::toggled,
    this, &TopologyEditorPanel::setAddEdgeMode);
  connect(
    move_vertex_mode_button_, &QPushButton::toggled,
    this, &TopologyEditorPanel::setMoveVertexMode);
  connect(
    delete_vertex_button_, &QPushButton::clicked,
    this, &TopologyEditorPanel::deleteSelectedVertex);
  connect(
    delete_edge_button_, &QPushButton::clicked,
    this, &TopologyEditorPanel::deleteSelectedEdge);
  connect(apply_button, &QPushButton::clicked, this, &TopologyEditorPanel::applyChanges);
  connect(
    batch_apply_button_, &QPushButton::clicked,
    this, &TopologyEditorPanel::applyBatchChanges);
  connect(
    batch_start_button_, &QPushButton::clicked,
    this, &TopologyEditorPanel::setBatchStart);
  connect(
    batch_end_button_, &QPushButton::clicked,
    this, &TopologyEditorPanel::setBatchEnd);
  connect(
    batch_clear_button_, &QPushButton::clicked,
    this, &TopologyEditorPanel::clearBatchSelection);
  connect(batch_pick_enabled_, &QCheckBox::toggled, this, [this](bool enabled) {
    if (enabled) {
      add_vertex_mode_button_->setChecked(false);
      add_edge_mode_button_->setChecked(false);
      move_vertex_mode_button_->setChecked(false);
      pending_edge_start_vertex_id_ = -1;
      moving_vertex_id_ = -1;
      setStatus("批量点选已开启：按 S 后依次点击起点和终点");
    }
  });
  connect(
    kind_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &TopologyEditorPanel::selectionKindChanged);
  connect(
    id_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &TopologyEditorPanel::selectionChanged);
}

QWidget * TopologyEditorPanel::buildVertexEditor()
{
  auto * widget = new QWidget();
  auto * form = new QFormLayout(widget);
  vertex_id_label_ = new QLabel("-");
  vertex_x_ = realBox();
  vertex_y_ = realBox();
  vertex_z_ = realBox();
  vertex_roll_ = realBox(-6.4, 6.4, 6);
  vertex_pitch_ = realBox(-6.4, 6.4, 6);
  vertex_yaw_ = realBox(-6.4, 6.4, 6);
  vertex_type_ = new QSpinBox();
  vertex_type_->setRange(0, 1000000);
  vertex_type_id_ = new QSpinBox();
  vertex_type_id_->setRange(0, 1000000);
  vertex_corner_ = new QCheckBox();
  vertex_slope_ = new QCheckBox();
  vertex_junction_ = new QCheckBox();
  vertex_charging_mode_ = new QSpinBox();
  vertex_charging_mode_->setRange(0, 100);
  vertex_acc_ = realBox(0.0, 100.0);
  vertex_turnable_ = new QCheckBox();
  vertex_align_yaw_ = new QCheckBox();
  vertex_must_pass_ = new QCheckBox();
  vertex_pass_radius_ = realBox(0.0, 100.0);
  vertex_pcd_ = new QLineEdit();
  form->addRow("顶点 ID（只读）", vertex_id_label_);
  form->addRow("X", vertex_x_);
  form->addRow("Y", vertex_y_);
  form->addRow("Z", vertex_z_);
  form->addRow("Roll (rad)", vertex_roll_);
  form->addRow("Pitch (rad)", vertex_pitch_);
  form->addRow("Yaw (rad)", vertex_yaw_);
  form->addRow("type", vertex_type_);
  form->addRow("typeId", vertex_type_id_);
  form->addRow("拐点 isCorner", vertex_corner_);
  form->addRow("坡点 isSlope", vertex_slope_);
  form->addRow("交汇点 isJunction", vertex_junction_);
  form->addRow("chargingMode", vertex_charging_mode_);
  form->addRow("acc", vertex_acc_);
  form->addRow("允许旋转 turnable", vertex_turnable_);
  form->addRow("终点对齐 alignFinalYaw", vertex_align_yaw_);
  form->addRow("必须经过 mustPassThrough", vertex_must_pass_);
  form->addRow("通过半径 passRadiusM", vertex_pass_radius_);
  form->addRow("PCD", vertex_pcd_);
  return widget;
}

QWidget * TopologyEditorPanel::buildEdgeEditor()
{
  auto * widget = new QWidget();
  auto * form = new QFormLayout(widget);
  edge_id_label_ = new QLabel("-");
  edge_endpoints_label_ = new QLabel("-");
  edge_weight_label_ = new QLabel("-");
  edge_source_label_ = new QLabel("-");
  edge_obstacle_mode_ = new QComboBox();
  edge_obstacle_mode_->addItem("0 · PID 三维扫掠停障", 0);
  edge_obstacle_mode_->addItem("1 · Efficient 3D 主动绕障", 1);
  edge_obstacle_mode_->addItem("2 · PID，不使用普通点云停障", 2);
  edge_obstacle_mode_->addItem("3 · 兼容忽略普通障碍", 3);
  edge_obstacle_mode_->addItem("4 · 外部栅格控制交接", 4);
  edge_controller_mode_ = new QComboBox();
  edge_controller_mode_->setEditable(true);
  edge_controller_mode_->addItems(
    {"auto", "pid", "local_planner", "efficient_3d_local_planner"});
  edge_travel_mode_ = new QComboBox();
  edge_travel_mode_->addItems({"bidirectional", "first_to_second", "second_to_first"});
  edge_rotation_allowed_ = new QCheckBox();
  edge_linear_speed_ = realBox(0.0, 100.0);
  edge_angular_speed_ = realBox(0.0, 100.0);
  edge_height_offset_ = realBox(-100.0, 100.0);
  edge_heading_angle_ = realBox(-6.4, 6.4, 6);
  edge_obstacle_box_ = new QLineEdit();
  edge_obstacle_box_->setPlaceholderText("前, 后, 左, 右");
  edge_grid_map_ = new QLineEdit();
  form->addRow("边 ID（只读）", edge_id_label_);
  form->addRow("端点（只读）", edge_endpoints_label_);
  form->addRow("长度（自动）", edge_weight_label_);
  form->addRow("来源（只读）", edge_source_label_);
  form->addRow("obstacleMode", edge_obstacle_mode_);
  form->addRow("controllerMode", edge_controller_mode_);
  form->addRow("travelMode", edge_travel_mode_);
  form->addRow("rotationAllowed", edge_rotation_allowed_);
  form->addRow("linearSpeedMps", edge_linear_speed_);
  form->addRow("angularSpeedRadps", edge_angular_speed_);
  form->addRow("heightOffsetM", edge_height_offset_);
  form->addRow("headingAngleRad", edge_heading_angle_);
  form->addRow("obstacleBoxM", edge_obstacle_box_);
  form->addRow("gridMapName", edge_grid_map_);
  return widget;
}

void TopologyEditorPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("无法获取 RViz ROS 节点", true);
    return;
  }
  node_ = abstraction->get_raw_node();
  marker_publisher_ = node_->create_publisher<MarkerArray>(
    "/route3d_topology_editor/visualization",
    rclcpp::QoS(1).reliable().transient_local());
  selection_subscription_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/route3d_topology_editor/select_point", rclcpp::QoS(10).reliable(),
    [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
      const double x = message->point.x;
      const double y = message->point.y;
      const double z = message->point.z;
      QMetaObject::invokeMethod(
        this, [this, x, y, z]() {selectNearest(x, y, z);}, Qt::QueuedConnection);
    });
  const std::string parameter_name = "topology_editor.graph_file";
  if (!node_->has_parameter(parameter_name)) {
    node_->declare_parameter<std::string>(parameter_name, "");
  }
  const auto parameter_path = node_->get_parameter(parameter_name).as_string();
  if (!parameter_path.empty()) {
    path_edit_->setText(QString::fromStdString(parameter_path));
    loadGraph();
  } else if (!path_edit_->text().trimmed().isEmpty()) {
    loadGraph();
  }
}

void TopologyEditorPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString saved_path;
  if (config.mapGetString("GraphFile", &saved_path) && path_edit_->text().isEmpty()) {
    path_edit_->setText(saved_path);
  }
}

void TopologyEditorPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("GraphFile", path_edit_->text());
}

void TopologyEditorPanel::browseGraph()
{
  const auto selected = QFileDialog::getOpenFileName(
    this, "打开 Route3D 拓扑", path_edit_->text(), "JSON (*.json)");
  if (!selected.isEmpty()) {
    path_edit_->setText(selected);
    loadGraph();
  }
}

void TopologyEditorPanel::loadGraph()
{
  if (dirty_ && QMessageBox::question(
      this, "放弃修改", "当前有未保存修改，确定重新加载吗？") != QMessageBox::Yes)
  {
    return;
  }
  try {
    document_.load(path_edit_->text().trimmed().toStdString());
    path_edit_->setText(QString::fromStdString(document_.path().string()));
    undo_stack_.clear();
    redo_stack_.clear();
    batch_start_vertex_id_ = -1;
    batch_end_vertex_id_ = -1;
    batch_vertex_ids_.clear();
    batch_edge_ids_.clear();
    batch_pick_enabled_->setChecked(false);
    clearGraphEditModes();
    loaded_ = true;
    setDirty(false);
    populateSelection();
    updateBatchUi();
    publishMarkers();
    setStatus(QString("已加载：%1 个点，%2 条边")
      .arg(document_.data().at("vertices").size())
      .arg(document_.data().at("edges").size()));
  } catch (const std::exception & error) {
    loaded_ = false;
    setLoadedState(false);
    setStatus(QString("加载失败：%1").arg(error.what()), true);
  }
}

bool TopologyEditorPanel::writeTo(const QString & path)
{
  try {
    document_.save(path.toStdString());
    document_.load(path.toStdString());
    path_edit_->setText(QString::fromStdString(document_.path().string()));
    setDirty(false);
    setStatus(QString("已保存：%1（原文件备份为 .bak）").arg(path));
    Q_EMIT configChanged();
    return true;
  } catch (const std::exception & error) {
    setStatus(QString("保存失败：%1").arg(error.what()), true);
    QMessageBox::critical(this, "保存失败", error.what());
    return false;
  }
}

void TopologyEditorPanel::saveGraph()
{
  if (loaded_) {
    (void)writeTo(path_edit_->text().trimmed());
  }
}

void TopologyEditorPanel::saveGraphAs()
{
  if (!loaded_) {
    return;
  }
  const auto selected = QFileDialog::getSaveFileName(
    this, "另存 Route3D 拓扑", path_edit_->text(), "JSON (*.json)");
  if (!selected.isEmpty()) {
    (void)writeTo(selected);
  }
}

void TopologyEditorPanel::selectionKindChanged(int index)
{
  editor_stack_->setCurrentIndex(index);
  populateBatchFields(index);
  batch_apply_button_->setText(
    index == 0 ? "批量应用所选顶点属性（不改坐标/RPY/PCD）" :
    "批量应用所选边属性");
  populateSelection();
}

void TopologyEditorPanel::populateBatchFields(int kind_index)
{
  if (!batch_field_combo_) {
    return;
  }
  batch_field_combo_->clear();
  if (kind_index == 0) {
    batch_field_combo_->addItem("拐点 isCorner", "isCorner");
    batch_field_combo_->addItem("坡点 isSlope", "isSlope");
    batch_field_combo_->addItem("交汇点 isJunction", "isJunction");
    batch_field_combo_->addItem("type", "type");
    batch_field_combo_->addItem("typeId", "typeId");
    batch_field_combo_->addItem("chargingMode", "chargingMode");
    batch_field_combo_->addItem("acc", "acc");
    batch_field_combo_->addItem("turnable", "turnable");
    batch_field_combo_->addItem("alignFinalYaw", "alignFinalYaw");
    batch_field_combo_->addItem("mustPassThrough", "mustPassThrough");
    batch_field_combo_->addItem("passRadiusM", "passRadiusM");
    batch_field_combo_->addItem("全部可批量顶点属性", "all");
  } else {
    batch_field_combo_->addItem("obstacleMode", "obstacleMode");
    batch_field_combo_->addItem("controllerMode", "controllerMode");
    batch_field_combo_->addItem("travelMode / dir", "travelMode");
    batch_field_combo_->addItem("rotationAllowed", "rotationAllowed");
    batch_field_combo_->addItem("linearSpeedMps", "linearSpeedMps");
    batch_field_combo_->addItem("angularSpeedRadps", "angularSpeedRadps");
    batch_field_combo_->addItem("heightOffsetM", "heightOffsetM");
    batch_field_combo_->addItem("headingAngleRad", "headingAngleRad");
    batch_field_combo_->addItem("obstacleBoxM", "obstacleBoxM");
    batch_field_combo_->addItem("gridMapName", "gridMapName");
    batch_field_combo_->addItem("全部可批量边属性", "all");
  }
}

void TopologyEditorPanel::selectionChanged(int index)
{
  if (index >= 0) {
    loadSelectedElement();
    publishMarkers();
  }
}

int TopologyEditorPanel::selectedId() const
{
  return id_combo_->currentIndex() < 0 ? -1 : id_combo_->currentData().toInt();
}

void TopologyEditorPanel::populateSelection(int preferred_id)
{
  id_combo_->blockSignals(true);
  id_combo_->clear();
  if (loaded_) {
    const char * section = kind_combo_->currentIndex() == 0 ? "vertices" : "edges";
    for (const int id : sortedIds(document_.data().at(section))) {
      id_combo_->addItem(
        QString(kind_combo_->currentIndex() == 0 ? "V%1" : "E%1").arg(id), id);
    }
    if (preferred_id > 0) {
      const int index = id_combo_->findData(preferred_id);
      if (index >= 0) {
        id_combo_->setCurrentIndex(index);
      }
    }
  }
  id_combo_->blockSignals(false);
  loadSelectedElement();
  setLoadedState(loaded_);
}

void TopologyEditorPanel::loadSelectedElement()
{
  if (!loaded_ || selectedId() < 1) {
    return;
  }
  const int id = selectedId();
  if (kind_combo_->currentIndex() == 0) {
    const auto & vertex = document_.data().at("vertices").at(std::to_string(id));
    const auto & meta = vertex.at("meta");
    vertex_id_label_->setText(QString::number(id));
    vertex_x_->setValue(vertex.at("pos").at(0).get<double>());
    vertex_y_->setValue(vertex.at("pos").at(1).get<double>());
    vertex_z_->setValue(vertex.at("pos").at(2).get<double>());
    vertex_roll_->setValue(vertex.at("rpy").at(0).get<double>());
    vertex_pitch_->setValue(vertex.at("rpy").at(1).get<double>());
    vertex_yaw_->setValue(vertex.at("rpy").at(2).get<double>());
    vertex_type_->setValue(meta.value("type", 0));
    vertex_type_id_->setValue(meta.value("typeId", 0));
    vertex_corner_->setChecked(meta.value("isCorner", false));
    vertex_slope_->setChecked(meta.value("isSlope", false));
    vertex_junction_->setChecked(meta.value("isJunction", false));
    vertex_charging_mode_->setValue(meta.value("chargingMode", 0));
    vertex_acc_->setValue(vertex.value("acc", 0.5));
    vertex_turnable_->setChecked(vertex.value("turnable", true));
    vertex_align_yaw_->setChecked(vertex.value("alignFinalYaw", false));
    vertex_must_pass_->setChecked(vertex.value("mustPassThrough", false));
    vertex_pass_radius_->setValue(vertex.value("passRadiusM", 0.45));
    vertex_pcd_->setText(QString::fromStdString(vertex.value("pcd", "")));
  } else {
    const auto & edge = document_.data().at("edges").at(std::to_string(id));
    const auto & meta = edge.at("meta");
    edge_id_label_->setText(QString::number(id));
    edge_endpoints_label_->setText(QString("%1 ↔ %2")
      .arg(edge.at("v").at(0).get<int>()).arg(edge.at("v").at(1).get<int>()));
    edge_weight_label_->setText(QString::number(edge.value("weight", 0.0), 'f', 3));
    edge_source_label_->setText(QString::fromStdString(meta.value("source", "unknown")));
    edge_obstacle_mode_->setCurrentIndex(std::clamp(meta.value("obstacleMode", 0), 0, 4));
    edge_controller_mode_->setCurrentText(
      QString::fromStdString(meta.value("controllerMode", "auto")));
    edge_travel_mode_->setCurrentText(
      QString::fromStdString(meta.value("travelMode", "bidirectional")));
    edge_rotation_allowed_->setChecked(edge.value("rotationAllowed", true));
    edge_linear_speed_->setValue(meta.value("linearSpeedMps", 1.0));
    edge_angular_speed_->setValue(meta.value("angularSpeedRadps", 0.0));
    edge_height_offset_->setValue(meta.value("heightOffsetM", 0.0));
    edge_heading_angle_->setValue(meta.value("headingAngleRad", 0.0));
    const auto box = meta.value("obstacleBoxM", json::array({0.0, 0.0, 0.0, 0.0}));
    QStringList box_text;
    if (box.is_array()) {
      for (const auto & item : box) {
        box_text.append(QString::number(item.get<double>(), 'g', 6));
      }
    }
    edge_obstacle_box_->setText(box_text.join(", "));
    edge_grid_map_->setText(QString::fromStdString(meta.value("gridMapName", "")));
  }
}

void TopologyEditorPanel::applyVertexAttributeFields(json & vertex) const
{
  auto & meta = vertex["meta"];
  meta["type"] = vertex_type_->value();
  meta["typeId"] = vertex_type_id_->value();
  meta["isCorner"] = vertex_corner_->isChecked();
  meta["isSlope"] = vertex_slope_->isChecked();
  meta["isJunction"] = vertex_junction_->isChecked();
  meta["chargingMode"] = vertex_charging_mode_->value();
  vertex["acc"] = vertex_acc_->value();
  vertex["turnable"] = vertex_turnable_->isChecked();
  vertex["alignFinalYaw"] = vertex_align_yaw_->isChecked();
  vertex["mustPassThrough"] = vertex_must_pass_->isChecked();
  vertex["passRadiusM"] = vertex_pass_radius_->value();
}

void TopologyEditorPanel::applyEdgeAttributeFields(json & edge) const
{
  const auto parts = edge_obstacle_box_->text().split(
    QRegularExpression("[,\\s]+"), Qt::SkipEmptyParts);
  if (parts.size() != 4) {
    throw std::runtime_error("obstacleBoxM 必须包含 4 个数字");
  }
  json obstacle_box = json::array();
  for (const auto & part : parts) {
    bool okay = false;
    const double value = part.toDouble(&okay);
    if (!okay || !std::isfinite(value) || value < 0.0) {
      throw std::runtime_error("obstacleBoxM 必须是 4 个非负数字");
    }
    obstacle_box.push_back(value);
  }
  const auto travel_mode = edge_travel_mode_->currentText().toStdString();
  const int direction = travel_mode == "first_to_second" ? 1 :
    (travel_mode == "second_to_first" ? 2 : 0);
  auto & meta = edge["meta"];
  meta["obstacleMode"] = edge_obstacle_mode_->currentData().toInt();
  meta["controllerMode"] = edge_controller_mode_->currentText().trimmed().toStdString();
  meta["travelMode"] = travel_mode;
  meta["dir"] = direction;
  edge["rotationAllowed"] = edge_rotation_allowed_->isChecked();
  meta["linearSpeedMps"] = edge_linear_speed_->value();
  meta["angularSpeedRadps"] = edge_angular_speed_->value();
  meta["heightOffsetM"] = edge_height_offset_->value();
  meta["headingAngleRad"] = edge_heading_angle_->value();
  meta["obstacleBoxM"] = std::move(obstacle_box);
  meta["gridMapName"] = edge_grid_map_->text().trimmed().toStdString();
}

void TopologyEditorPanel::applyChanges()
{
  if (!loaded_ || selectedId() < 1) {
    return;
  }
  const int id = selectedId();
  const auto before = document_.data();
  try {
    if (kind_combo_->currentIndex() == 0) {
      auto & vertex = document_.data().at("vertices").at(std::to_string(id));
      vertex["pos"] = {vertex_x_->value(), vertex_y_->value(), vertex_z_->value()};
      vertex["rpy"] = {
        vertex_roll_->value(), vertex_pitch_->value(), vertex_yaw_->value()};
      applyVertexAttributeFields(vertex);
      vertex["pcd"] = vertex_pcd_->text().trimmed().toStdString();
      TopologyDocument::recomputeIncidentWeights(document_.data(), id);
    } else {
      auto & edge = document_.data().at("edges").at(std::to_string(id));
      applyEdgeAttributeFields(edge);
    }
    document_.validate();
    undo_stack_.push_back(before);
    if (undo_stack_.size() > 50U) {
      undo_stack_.erase(undo_stack_.begin());
    }
    redo_stack_.clear();
    document_.data()["version"] = document_.data().value("version", 0) + 1;
    setDirty(true);
    loadSelectedElement();
    publishMarkers();
    setStatus("修改已应用到内存；点击“保存”后才会写入文件");
  } catch (const std::exception & error) {
    document_.data() = before;
    setStatus(QString("修改失败：%1").arg(error.what()), true);
  }
}

void TopologyEditorPanel::applyAllStop()
{
  applyAllObstacleMode(0);
}

void TopologyEditorPanel::applyAllAvoid()
{
  applyAllObstacleMode(1);
}

void TopologyEditorPanel::applyAllObstacleMode(int mode)
{
  if (!loaded_) {
    return;
  }
  const auto edge_count = document_.data().at("edges").size();
  if (edge_count == 0U) {
    setStatus("当前拓扑没有边", true);
    return;
  }
  const QString action = mode == 0 ?
    "全图停障：obstacleMode=0，controllerMode=pid" :
    "全图避障：obstacleMode=1，controllerMode=efficient_3d_local_planner";
  if (QMessageBox::question(
      this, "确认全图修改",
      QString("将 %1 条边全部设置为：\n%2\n\n此操作可以撤销，点击保存后才写入文件。")
      .arg(edge_count).arg(action)) != QMessageBox::Yes)
  {
    return;
  }

  const auto before = document_.data();
  try {
    for (auto & [edge_id, edge] : document_.data().at("edges").items()) {
      (void)edge_id;
      edge["meta"]["obstacleMode"] = mode;
      edge["meta"]["controllerMode"] =
        mode == 0 ? "pid" : "efficient_3d_local_planner";
    }
    document_.validate();
    undo_stack_.push_back(before);
    if (undo_stack_.size() > 50U) {
      undo_stack_.erase(undo_stack_.begin());
    }
    redo_stack_.clear();
    document_.data()["version"] = document_.data().value("version", 0) + 1;
    setDirty(true);
    loadSelectedElement();
    publishMarkers();
    setStatus(QString("已应用%1到全部 %2 条边；点击“保存”后写入文件")
      .arg(mode == 0 ? "停障" : "避障").arg(edge_count));
  } catch (const std::exception & error) {
    document_.data() = before;
    setStatus(QString("全图修改失败：%1").arg(error.what()), true);
  }
}

void TopologyEditorPanel::setAddVertexMode(bool enabled)
{
  if (!enabled) {
    return;
  }
  if (!loaded_) {
    add_vertex_mode_button_->setChecked(false);
    return;
  }
  add_edge_mode_button_->setChecked(false);
  move_vertex_mode_button_->setChecked(false);
  batch_pick_enabled_->setChecked(false);
  pending_edge_start_vertex_id_ = -1;
  moving_vertex_id_ = -1;
  setStatus("添加点模式：按 S 后点击 RViz 中的新位置，可连续添加");
}

void TopologyEditorPanel::setAddEdgeMode(bool enabled)
{
  pending_edge_start_vertex_id_ = -1;
  if (!enabled) {
    publishMarkers();
    return;
  }
  if (!loaded_) {
    add_edge_mode_button_->setChecked(false);
    return;
  }
  add_vertex_mode_button_->setChecked(false);
  move_vertex_mode_button_->setChecked(false);
  batch_pick_enabled_->setChecked(false);
  moving_vertex_id_ = -1;
  publishMarkers();
  setStatus("连接两点模式：按 S 后依次点击两个已有顶点");
}

void TopologyEditorPanel::setMoveVertexMode(bool enabled)
{
  if (!enabled) {
    moving_vertex_id_ = -1;
    return;
  }
  if (!loaded_ || kind_combo_->currentIndex() != 0 || selectedId() < 1) {
    move_vertex_mode_button_->setChecked(false);
    setStatus("请先在“顶点”页选中需要移动的点", true);
    return;
  }
  moving_vertex_id_ = selectedId();
  add_vertex_mode_button_->setChecked(false);
  add_edge_mode_button_->setChecked(false);
  batch_pick_enabled_->setChecked(false);
  pending_edge_start_vertex_id_ = -1;
  setStatus(QString("移动 V%1：按 S 后点击新的 XY 位置，原 Z 高度保持不变")
    .arg(moving_vertex_id_));
}

void TopologyEditorPanel::clearGraphEditModes()
{
  add_vertex_mode_button_->setChecked(false);
  add_edge_mode_button_->setChecked(false);
  move_vertex_mode_button_->setChecked(false);
  pending_edge_start_vertex_id_ = -1;
  moving_vertex_id_ = -1;
}

void TopologyEditorPanel::addVertexAt(double x, double y, double z)
{
  const auto before = document_.data();
  try {
    for (const auto & [id, vertex] : document_.data().at("vertices").items()) {
      (void)id;
      const auto & position = vertex.at("pos");
      const double dx = x - position.at(0).get<double>();
      const double dy = y - position.at(1).get<double>();
      const double dz = z - position.at(2).get<double>();
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < 0.05) {
        throw std::runtime_error("新点与已有点距离小于 0.05 m");
      }
    }
    const int id = TopologyDocument::addVertex(document_.data(), x, y, z, node_->now().seconds());
    document_.validate();
    undo_stack_.push_back(before);
    if (undo_stack_.size() > 50U) {
      undo_stack_.erase(undo_stack_.begin());
    }
    redo_stack_.clear();
    document_.data()["version"] = document_.data().value("version", 0) + 1;
    setDirty(true);
    if (kind_combo_->currentIndex() != 0) {
      kind_combo_->setCurrentIndex(0);
    }
    populateSelection(id);
    publishMarkers();
    setStatus(QString("已添加 V%1（自动记点默认属性）；继续点击可添加更多点").arg(id));
  } catch (const std::exception & error) {
    document_.data() = before;
    setStatus(QString("添加点失败：%1").arg(error.what()), true);
  }
}

void TopologyEditorPanel::addEdgeTo(int vertex_id)
{
  if (pending_edge_start_vertex_id_ < 1) {
    pending_edge_start_vertex_id_ = vertex_id;
    publishMarkers();
    setStatus(QString("连接起点为 V%1，请点击第二个顶点").arg(vertex_id));
    return;
  }
  const int first = pending_edge_start_vertex_id_;
  const auto before = document_.data();
  try {
    const int edge_id = TopologyDocument::addEdge(document_.data(), first, vertex_id);
    document_.validate();
    undo_stack_.push_back(before);
    if (undo_stack_.size() > 50U) {
      undo_stack_.erase(undo_stack_.begin());
    }
    redo_stack_.clear();
    document_.data()["version"] = document_.data().value("version", 0) + 1;
    setDirty(true);
    pending_edge_start_vertex_id_ = -1;
    kind_combo_->setCurrentIndex(1);
    populateSelection(edge_id);
    if (batch_start_vertex_id_ > 0 && batch_end_vertex_id_ > 0) {
      recomputeBatchPath();
    } else {
      publishMarkers();
    }
    setStatus(QString("已添加 E%1：V%2 ↔ V%3，默认 obstacleMode=0；可继续连接")
      .arg(edge_id).arg(first).arg(vertex_id));
  } catch (const std::exception & error) {
    document_.data() = before;
    setStatus(QString("添加边失败：%1").arg(error.what()), true);
  }
}

void TopologyEditorPanel::moveVertexTo(double x, double y)
{
  const int id = moving_vertex_id_;
  const auto before = document_.data();
  try {
    auto & position = document_.data().at("vertices").at(std::to_string(id)).at("pos");
    position[0] = x;
    position[1] = y;
    TopologyDocument::recomputeIncidentWeights(document_.data(), id);
    document_.validate();
    undo_stack_.push_back(before);
    if (undo_stack_.size() > 50U) {
      undo_stack_.erase(undo_stack_.begin());
    }
    redo_stack_.clear();
    document_.data()["version"] = document_.data().value("version", 0) + 1;
    setDirty(true);
    move_vertex_mode_button_->setChecked(false);
    if (kind_combo_->currentIndex() != 0) {
      kind_combo_->setCurrentIndex(0);
    }
    populateSelection(id);
    recomputeBatchPath();
    publishMarkers();
    setStatus(QString("已移动 V%1 的 XY 并重算关联边长度；Z 保持不变").arg(id));
  } catch (const std::exception & error) {
    document_.data() = before;
    move_vertex_mode_button_->setChecked(false);
    setStatus(QString("移动点失败：%1").arg(error.what()), true);
  }
}

void TopologyEditorPanel::deleteSelectedVertex()
{
  if (!loaded_ || kind_combo_->currentIndex() != 0 || selectedId() < 1) {
    setStatus("请先在“顶点”页选中需要删除的点", true);
    return;
  }
  const int id = selectedId();
  std::size_t incident_count = 0;
  for (const auto & [edge_id, edge] : document_.data().at("edges").items()) {
    (void)edge_id;
    if (edge.at("v").at(0).get<int>() == id || edge.at("v").at(1).get<int>() == id) {
      ++incident_count;
    }
  }
  if (QMessageBox::question(
      this, "确认删除顶点",
      QString("删除 V%1，同时删除与它连接的 %2 条边？\n此操作可以撤销。")
      .arg(id).arg(incident_count)) != QMessageBox::Yes)
  {
    return;
  }
  const auto before = document_.data();
  try {
    TopologyDocument::removeVertex(document_.data(), id);
    document_.validate();
    undo_stack_.push_back(before);
    if (undo_stack_.size() > 50U) {
      undo_stack_.erase(undo_stack_.begin());
    }
    redo_stack_.clear();
    document_.data()["version"] = document_.data().value("version", 0) + 1;
    setDirty(true);
    clearGraphEditModes();
    batch_start_vertex_id_ = batch_end_vertex_id_ = -1;
    batch_vertex_ids_.clear();
    batch_edge_ids_.clear();
    updateBatchUi();
    populateSelection();
    publishMarkers();
    setStatus(QString("已删除 V%1 及其 %2 条关联边；点击“保存”后写入文件")
      .arg(id).arg(incident_count));
  } catch (const std::exception & error) {
    document_.data() = before;
    setStatus(QString("删除点失败：%1").arg(error.what()), true);
  }
}

void TopologyEditorPanel::deleteSelectedEdge()
{
  if (!loaded_ || kind_combo_->currentIndex() != 1 || selectedId() < 1) {
    setStatus("请先在“边”页选中需要删除的边", true);
    return;
  }
  const int id = selectedId();
  if (QMessageBox::question(
      this, "确认删除边", QString("删除 E%1？\n此操作可以撤销。").arg(id)) !=
    QMessageBox::Yes)
  {
    return;
  }
  const auto before = document_.data();
  try {
    TopologyDocument::removeEdge(document_.data(), id);
    document_.validate();
    undo_stack_.push_back(before);
    if (undo_stack_.size() > 50U) {
      undo_stack_.erase(undo_stack_.begin());
    }
    redo_stack_.clear();
    document_.data()["version"] = document_.data().value("version", 0) + 1;
    setDirty(true);
    clearGraphEditModes();
    batch_start_vertex_id_ = batch_end_vertex_id_ = -1;
    batch_vertex_ids_.clear();
    batch_edge_ids_.clear();
    updateBatchUi();
    populateSelection();
    publishMarkers();
    setStatus(QString("已删除 E%1；点击“保存”后写入文件").arg(id));
  } catch (const std::exception & error) {
    document_.data() = before;
    setStatus(QString("删除边失败：%1").arg(error.what()), true);
  }
}

void TopologyEditorPanel::setBatchStart()
{
  if (!loaded_ || kind_combo_->currentIndex() != 0 || selectedId() < 1) {
    setStatus("请先切换到“顶点”并选中批量起点", true);
    return;
  }
  batch_start_vertex_id_ = selectedId();
  batch_end_vertex_id_ = -1;
  batch_vertex_ids_.clear();
  batch_edge_ids_.clear();
  updateBatchUi();
  publishMarkers();
  setStatus(QString("已设置批量起点 V%1，请选择终点").arg(batch_start_vertex_id_));
}

void TopologyEditorPanel::setBatchEnd()
{
  if (!loaded_ || kind_combo_->currentIndex() != 0 || selectedId() < 1) {
    setStatus("请先切换到“顶点”并选中批量终点", true);
    return;
  }
  if (batch_start_vertex_id_ < 1) {
    setStatus("请先设置批量起点", true);
    return;
  }
  batch_end_vertex_id_ = selectedId();
  recomputeBatchPath();
}

void TopologyEditorPanel::clearBatchSelection()
{
  batch_start_vertex_id_ = -1;
  batch_end_vertex_id_ = -1;
  batch_vertex_ids_.clear();
  batch_edge_ids_.clear();
  batch_pick_enabled_->setChecked(false);
  updateBatchUi();
  publishMarkers();
  setStatus("已清除批量范围");
}

void TopologyEditorPanel::recomputeBatchPath()
{
  batch_vertex_ids_.clear();
  batch_edge_ids_.clear();
  if (batch_start_vertex_id_ < 1 || batch_end_vertex_id_ < 1) {
    updateBatchUi();
    return;
  }
  try {
    const auto path = TopologyDocument::shortestPath(
      document_.data(), batch_start_vertex_id_, batch_end_vertex_id_);
    batch_vertex_ids_ = path.vertex_ids;
    batch_edge_ids_ = path.edge_ids;
    updateBatchUi();
    publishMarkers();
    setStatus(QString("批量路径已选定：%1 个点、%2 条边")
      .arg(batch_vertex_ids_.size()).arg(batch_edge_ids_.size()));
  } catch (const std::exception & error) {
    batch_end_vertex_id_ = -1;
    updateBatchUi();
    publishMarkers();
    setStatus(QString("批量路径选择失败：%1").arg(error.what()), true);
  }
}

void TopologyEditorPanel::updateBatchUi()
{
  QString text = "批量范围：";
  if (batch_start_vertex_id_ < 1) {
    text += "未选择（起终点间最短拓扑路径）";
  } else if (batch_end_vertex_id_ < 1) {
    text += QString("起点 V%1，等待终点").arg(batch_start_vertex_id_);
  } else {
    text += QString("V%1 → V%2，共 %3 个点、%4 条边")
      .arg(batch_start_vertex_id_).arg(batch_end_vertex_id_)
      .arg(batch_vertex_ids_.size()).arg(batch_edge_ids_.size());
  }
  batch_status_label_->setText(text);
  setLoadedState(loaded_);
}

void TopologyEditorPanel::applyBatchChanges()
{
  if (!loaded_ || batch_vertex_ids_.empty() || batch_end_vertex_id_ < 1) {
    setStatus("请先点选批量起点和终点", true);
    return;
  }
  const bool editing_vertices = kind_combo_->currentIndex() == 0;
  const std::size_t count = editing_vertices ? batch_vertex_ids_.size() : batch_edge_ids_.size();
  if (count == 0U) {
    setStatus("当前批量路径中没有可修改的边", true);
    return;
  }
  const QString field_label = batch_field_combo_->currentText();
  const std::string field = batch_field_combo_->currentData().toString().toStdString();
  const QString target = editing_vertices ? "个顶点" : "条边";
  if (QMessageBox::question(
      this, "确认批量修改",
      QString("将字段“%1”的当前表单值应用到最短路径上的 %2 %3？\n此操作可以撤销。")
      .arg(field_label).arg(count).arg(target)) != QMessageBox::Yes)
  {
    return;
  }

  const auto before = document_.data();
  try {
    if (editing_vertices) {
      json staged = document_.data().at("vertices").at(std::to_string(selectedId()));
      applyVertexAttributeFields(staged);
      for (const int id : batch_vertex_ids_) {
        auto & vertex = document_.data().at("vertices").at(std::to_string(id));
        if (field == "all") {
          applyVertexAttributeFields(vertex);
        } else if (
          field == "acc" || field == "turnable" || field == "alignFinalYaw" ||
          field == "mustPassThrough" || field == "passRadiusM")
        {
          vertex[field] = staged.at(field);
        } else {
          vertex["meta"][field] = staged.at("meta").at(field);
        }
      }
    } else {
      json staged = document_.data().at("edges").at(std::to_string(selectedId()));
      applyEdgeAttributeFields(staged);
      for (const int id : batch_edge_ids_) {
        auto & edge = document_.data().at("edges").at(std::to_string(id));
        if (field == "all") {
          applyEdgeAttributeFields(edge);
        } else if (field == "rotationAllowed") {
          edge[field] = staged.at(field);
        } else if (field == "travelMode") {
          edge["meta"]["travelMode"] = staged.at("meta").at("travelMode");
          edge["meta"]["dir"] = staged.at("meta").at("dir");
        } else {
          edge["meta"][field] = staged.at("meta").at(field);
        }
      }
    }
    document_.validate();
    undo_stack_.push_back(before);
    if (undo_stack_.size() > 50U) {
      undo_stack_.erase(undo_stack_.begin());
    }
    redo_stack_.clear();
    document_.data()["version"] = document_.data().value("version", 0) + 1;
    setDirty(true);
    loadSelectedElement();
    publishMarkers();
    setStatus(QString("已把“%1”批量应用到 %2 %3；点击“保存”后写入文件")
      .arg(field_label).arg(count).arg(target));
  } catch (const std::exception & error) {
    document_.data() = before;
    setStatus(QString("批量修改失败：%1").arg(error.what()), true);
  }
}

void TopologyEditorPanel::undo()
{
  if (!loaded_ || undo_stack_.empty()) {
    return;
  }
  const int id = selectedId();
  redo_stack_.push_back(document_.data());
  document_.data() = std::move(undo_stack_.back());
  undo_stack_.pop_back();
  setDirty(true);
  populateSelection(id);
  publishMarkers();
  setStatus("已撤销；尚未写入文件");
}

void TopologyEditorPanel::redo()
{
  if (!loaded_ || redo_stack_.empty()) {
    return;
  }
  const int id = selectedId();
  undo_stack_.push_back(document_.data());
  document_.data() = std::move(redo_stack_.back());
  redo_stack_.pop_back();
  setDirty(true);
  populateSelection(id);
  publishMarkers();
  setStatus("已重做；尚未写入文件");
}

void TopologyEditorPanel::setLoadedState(bool loaded)
{
  save_button_->setEnabled(loaded);
  save_as_button_->setEnabled(loaded);
  all_stop_button_->setEnabled(loaded);
  all_avoid_button_->setEnabled(loaded);
  add_vertex_mode_button_->setEnabled(loaded);
  add_edge_mode_button_->setEnabled(loaded);
  move_vertex_mode_button_->setEnabled(loaded);
  delete_vertex_button_->setEnabled(loaded);
  delete_edge_button_->setEnabled(loaded);
  kind_combo_->setEnabled(loaded);
  id_combo_->setEnabled(loaded);
  batch_pick_enabled_->setEnabled(loaded);
  batch_start_button_->setEnabled(loaded);
  batch_end_button_->setEnabled(loaded);
  batch_clear_button_->setEnabled(loaded);
  batch_field_combo_->setEnabled(loaded);
  batch_apply_button_->setEnabled(
    loaded && batch_start_vertex_id_ > 0 && batch_end_vertex_id_ > 0 &&
    !batch_vertex_ids_.empty());
  editor_stack_->setEnabled(loaded);
  undo_button_->setEnabled(loaded && !undo_stack_.empty());
  redo_button_->setEnabled(loaded && !redo_stack_.empty());
}

void TopologyEditorPanel::setDirty(bool dirty)
{
  dirty_ = dirty;
  save_button_->setText(dirty ? "保存 *" : "保存");
  setLoadedState(loaded_);
}

void TopologyEditorPanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "QLabel { color: #ff6666; }" : "");
}

void TopologyEditorPanel::publishMarkers()
{
  if (!loaded_ || !marker_publisher_ || !node_) {
    return;
  }
  const auto & data = document_.data();
  const auto & vertices = data.at("vertices");
  const auto stamp = node_->now();
  const std::string frame_id = data.value("frame_id", "camera_init");
  MarkerArray output;
  Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = stamp;
  clear.action = Marker::DELETEALL;
  output.markers.push_back(clear);

  auto points = marker(frame_id, "vertices", 0, Marker::SPHERE_LIST, stamp);
  points.scale.x = points.scale.y = points.scale.z = 0.20;
  points.color.a = 1.0F;
  for (const int id : sortedIds(vertices)) {
    const auto & vertex = vertices.at(std::to_string(id));
    points.points.push_back(point(vertex.at("pos"), 0.12));
    points.colors.push_back(vertexColor(vertex));
    auto label = marker(frame_id, "vertex_ids", id, Marker::TEXT_VIEW_FACING, stamp);
    label.pose.position = point(vertex.at("pos"), 0.34);
    label.scale.z = 0.16;
    label.color = vertexColor(vertex);
    label.text = "V" + std::to_string(id);
    output.markers.push_back(std::move(label));
  }
  output.markers.push_back(std::move(points));

  auto edges = marker(frame_id, "edges", 0, Marker::LINE_LIST, stamp);
  edges.scale.x = 0.075;
  edges.color.a = 1.0F;
  for (const int id : sortedIds(data.at("edges"))) {
    const auto & edge = data.at("edges").at(std::to_string(id));
    const int first_id = edge.at("v").at(0).get<int>();
    const int second_id = edge.at("v").at(1).get<int>();
    const auto & first = vertices.at(std::to_string(first_id)).at("pos");
    const auto & second = vertices.at(std::to_string(second_id)).at("pos");
    const auto edge_color = edge.at("meta").value("source", "") == "loop_closure" ?
      rgba(0.0F, 1.0F, 1.0F) : edgeColor(edge.at("meta").value("obstacleMode", 0));
    edges.points.push_back(point(first, 0.08));
    edges.points.push_back(point(second, 0.08));
    edges.colors.push_back(edge_color);
    edges.colors.push_back(edge_color);
    auto label = marker(frame_id, "edge_ids", id, Marker::TEXT_VIEW_FACING, stamp);
    label.pose.position.x = 0.5 * (first.at(0).get<double>() + second.at(0).get<double>());
    label.pose.position.y = 0.5 * (first.at(1).get<double>() + second.at(1).get<double>());
    label.pose.position.z = 0.5 * (first.at(2).get<double>() + second.at(2).get<double>()) +
      0.22;
    label.scale.z = 0.15;
    label.color = edge_color;
    label.text = "E" + std::to_string(id);
    output.markers.push_back(std::move(label));
  }
  output.markers.push_back(std::move(edges));

  if (!batch_vertex_ids_.empty()) {
    auto batch_edges = marker(frame_id, "batch_path", 0, Marker::LINE_LIST, stamp);
    batch_edges.scale.x = 0.16;
    batch_edges.color = rgba(1.0F, 0.68F, 0.05F, 0.95F);
    for (const int edge_id : batch_edge_ids_) {
      const auto & edge = data.at("edges").at(std::to_string(edge_id));
      const auto & first = vertices.at(
        std::to_string(edge.at("v").at(0).get<int>())).at("pos");
      const auto & second = vertices.at(
        std::to_string(edge.at("v").at(1).get<int>())).at("pos");
      batch_edges.points.push_back(point(first, 0.16));
      batch_edges.points.push_back(point(second, 0.16));
    }
    output.markers.push_back(std::move(batch_edges));

    auto batch_vertices = marker(frame_id, "batch_vertices", 0, Marker::SPHERE_LIST, stamp);
    batch_vertices.scale.x = batch_vertices.scale.y = batch_vertices.scale.z = 0.28;
    batch_vertices.color = rgba(1.0F, 0.68F, 0.05F, 0.95F);
    for (const int vertex_id : batch_vertex_ids_) {
      batch_vertices.points.push_back(
        point(vertices.at(std::to_string(vertex_id)).at("pos"), 0.18));
    }
    output.markers.push_back(std::move(batch_vertices));

    auto start_marker = marker(frame_id, "batch_endpoints", 0, Marker::SPHERE, stamp);
    start_marker.pose.position = point(
      vertices.at(std::to_string(batch_start_vertex_id_)).at("pos"), 0.18);
    start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.42;
    start_marker.color = rgba(0.15F, 1.0F, 0.25F, 0.95F);
    output.markers.push_back(std::move(start_marker));
    if (batch_end_vertex_id_ > 0) {
      auto end_marker = marker(frame_id, "batch_endpoints", 1, Marker::SPHERE, stamp);
      end_marker.pose.position = point(
        vertices.at(std::to_string(batch_end_vertex_id_)).at("pos"), 0.18);
      end_marker.scale.x = end_marker.scale.y = end_marker.scale.z = 0.42;
      end_marker.color = rgba(1.0F, 0.18F, 0.12F, 0.95F);
      output.markers.push_back(std::move(end_marker));
    }
  } else if (batch_start_vertex_id_ > 0) {
    auto start_marker = marker(frame_id, "batch_endpoints", 0, Marker::SPHERE, stamp);
    start_marker.pose.position = point(
      vertices.at(std::to_string(batch_start_vertex_id_)).at("pos"), 0.18);
    start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.42;
    start_marker.color = rgba(0.15F, 1.0F, 0.25F, 0.95F);
    output.markers.push_back(std::move(start_marker));
  }

  if (pending_edge_start_vertex_id_ > 0 &&
    vertices.contains(std::to_string(pending_edge_start_vertex_id_)))
  {
    auto edge_start = marker(frame_id, "new_edge_start", 0, Marker::SPHERE, stamp);
    edge_start.pose.position = point(
      vertices.at(std::to_string(pending_edge_start_vertex_id_)).at("pos"), 0.20);
    edge_start.scale.x = edge_start.scale.y = edge_start.scale.z = 0.46;
    edge_start.color = rgba(0.15F, 1.0F, 0.25F, 0.95F);
    output.markers.push_back(std::move(edge_start));
  }

  const int selected = selectedId();
  if (selected > 0 && kind_combo_->currentIndex() == 0) {
    const auto & vertex = vertices.at(std::to_string(selected));
    auto highlight = marker(frame_id, "selection", 0, Marker::SPHERE, stamp);
    highlight.pose.position = point(vertex.at("pos"), 0.12);
    highlight.scale.x = highlight.scale.y = highlight.scale.z = 0.34;
    highlight.color = rgba(1.0F, 1.0F, 1.0F, 0.85F);
    output.markers.push_back(std::move(highlight));
  } else if (selected > 0 && kind_combo_->currentIndex() == 1) {
    const auto & edge = data.at("edges").at(std::to_string(selected));
    const auto & first = vertices.at(std::to_string(edge.at("v").at(0).get<int>())).at("pos");
    const auto & second = vertices.at(std::to_string(edge.at("v").at(1).get<int>())).at("pos");
    auto highlight = marker(frame_id, "selection", 0, Marker::LINE_LIST, stamp);
    highlight.scale.x = 0.18;
    highlight.color = rgba(1.0F, 1.0F, 1.0F);
    highlight.points.push_back(point(first, 0.14));
    highlight.points.push_back(point(second, 0.14));
    output.markers.push_back(std::move(highlight));
  }
  marker_publisher_->publish(output);
}

void TopologyEditorPanel::selectNearest(double x, double y, double z)
{
  if (!loaded_) {
    return;
  }
  if (add_vertex_mode_button_->isChecked()) {
    addVertexAt(x, y, z);
    return;
  }
  if (move_vertex_mode_button_->isChecked() && moving_vertex_id_ > 0) {
    moveVertexTo(x, y);
    return;
  }
  const auto & data = document_.data();
  const auto & vertices = data.at("vertices");
  int nearest_id = -1;
  double nearest_distance = std::numeric_limits<double>::infinity();

  const bool select_vertex = batch_pick_enabled_->isChecked() ||
    add_edge_mode_button_->isChecked() || kind_combo_->currentIndex() == 0;
  if (select_vertex) {
    for (const int id : sortedIds(vertices)) {
      const auto & position = vertices.at(std::to_string(id)).at("pos");
      const double dx = x - position.at(0).get<double>();
      const double dy = y - position.at(1).get<double>();
      const double dz = z - position.at(2).get<double>();
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_id = id;
      }
    }
  } else {
    for (const int id : sortedIds(data.at("edges"))) {
      const auto & edge = data.at("edges").at(std::to_string(id));
      const auto & first = vertices.at(
        std::to_string(edge.at("v").at(0).get<int>())).at("pos");
      const auto & second = vertices.at(
        std::to_string(edge.at("v").at(1).get<int>())).at("pos");
      const std::array<double, 3> start = {
        first.at(0).get<double>(), first.at(1).get<double>(), first.at(2).get<double>()};
      const std::array<double, 3> delta = {
        second.at(0).get<double>() - start[0],
        second.at(1).get<double>() - start[1],
        second.at(2).get<double>() - start[2]};
      const double denominator =
        delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
      const double ratio = denominator <= 1.0e-12 ? 0.0 : std::clamp(
        ((x - start[0]) * delta[0] + (y - start[1]) * delta[1] +
        (z - start[2]) * delta[2]) / denominator, 0.0, 1.0);
      const double dx = x - (start[0] + ratio * delta[0]);
      const double dy = y - (start[1] + ratio * delta[1]);
      const double dz = z - (start[2] + ratio * delta[2]);
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_id = id;
      }
    }
  }

  if (nearest_id < 1 || nearest_distance > 1.0) {
    setStatus(
      select_vertex ? "点击位置 1 m 内没有拓扑顶点" :
      "点击位置 1 m 内没有拓扑边", true);
    return;
  }
  if (add_edge_mode_button_->isChecked()) {
    if (kind_combo_->currentIndex() != 0) {
      kind_combo_->setCurrentIndex(0);
    }
    const int vertex_index = id_combo_->findData(nearest_id);
    if (vertex_index >= 0) {
      id_combo_->setCurrentIndex(vertex_index);
    }
    addEdgeTo(nearest_id);
    return;
  }
  if (batch_pick_enabled_->isChecked()) {
    if (kind_combo_->currentIndex() != 0) {
      kind_combo_->setCurrentIndex(0);
    }
    const int vertex_index = id_combo_->findData(nearest_id);
    if (vertex_index >= 0) {
      id_combo_->setCurrentIndex(vertex_index);
    }
    if (batch_start_vertex_id_ < 1 || batch_end_vertex_id_ > 0) {
      batch_start_vertex_id_ = nearest_id;
      batch_end_vertex_id_ = -1;
      batch_vertex_ids_.clear();
      batch_edge_ids_.clear();
      updateBatchUi();
      publishMarkers();
      setStatus(QString("已点选批量起点 V%1，请继续点击终点").arg(nearest_id));
    } else {
      batch_end_vertex_id_ = nearest_id;
      batch_pick_enabled_->setChecked(false);
      recomputeBatchPath();
    }
    return;
  }
  const int index = id_combo_->findData(nearest_id);
  if (index >= 0) {
    id_combo_->setCurrentIndex(index);
    setStatus(QString("已选中 %1%2，点击距离 %3 m")
      .arg(select_vertex ? "V" : "E")
      .arg(nearest_id).arg(nearest_distance, 0, 'f', 3));
  }
}

}  // namespace route3d_topology_editor

PLUGINLIB_EXPORT_CLASS(route3d_topology_editor::TopologyEditorPanel, rviz_common::Panel)
