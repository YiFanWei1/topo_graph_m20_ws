#ifndef ROUTE3D_TOPOLOGY_EDITOR__TOPOLOGY_EDITOR_PANEL_HPP_
#define ROUTE3D_TOPOLOGY_EDITOR__TOPOLOGY_EDITOR_PANEL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "route3d_topology_editor/topology_document.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QStackedWidget;

namespace route3d_topology_editor
{

class TopologyEditorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TopologyEditorPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void browseGraph();
  void loadGraph();
  void saveGraph();
  void saveGraphAs();
  void selectionKindChanged(int index);
  void selectionChanged(int index);
  void applyChanges();
  void applyBatchChanges();
  void applyAllStop();
  void applyAllAvoid();
  void setAddVertexMode(bool enabled);
  void setAddEdgeMode(bool enabled);
  void setMoveVertexMode(bool enabled);
  void deleteSelectedVertex();
  void deleteSelectedEdge();
  void setBatchStart();
  void setBatchEnd();
  void clearBatchSelection();
  void undo();
  void redo();

private:
  void buildUi();
  QWidget * buildVertexEditor();
  QWidget * buildEdgeEditor();
  void populateSelection(int preferred_id = -1);
  void loadSelectedElement();
  void setLoadedState(bool loaded);
  void setDirty(bool dirty);
  void setStatus(const QString & text, bool error = false);
  void publishMarkers();
  void selectNearest(double x, double y, double z);
  void recomputeBatchPath();
  void updateBatchUi();
  void populateBatchFields(int kind_index);
  void applyVertexAttributeFields(nlohmann::json & vertex) const;
  void applyEdgeAttributeFields(nlohmann::json & edge) const;
  void applyAllObstacleMode(int mode);
  void addVertexAt(double x, double y, double z);
  void addEdgeTo(int vertex_id);
  void moveVertexTo(double x, double y);
  void clearGraphEditModes();
  bool writeTo(const QString & path);
  int selectedId() const;

  TopologyDocument document_;
  bool loaded_{false};
  bool dirty_{false};
  std::vector<nlohmann::json> undo_stack_;
  std::vector<nlohmann::json> redo_stack_;
  int batch_start_vertex_id_{-1};
  int batch_end_vertex_id_{-1};
  int pending_edge_start_vertex_id_{-1};
  int moving_vertex_id_{-1};
  std::vector<int> batch_vertex_ids_;
  std::vector<int> batch_edge_ids_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr selection_subscription_;

  QLineEdit * path_edit_{nullptr};
  QPushButton * load_button_{nullptr};
  QPushButton * save_button_{nullptr};
  QPushButton * save_as_button_{nullptr};
  QPushButton * undo_button_{nullptr};
  QPushButton * redo_button_{nullptr};
  QPushButton * all_stop_button_{nullptr};
  QPushButton * all_avoid_button_{nullptr};
  QPushButton * add_vertex_mode_button_{nullptr};
  QPushButton * add_edge_mode_button_{nullptr};
  QPushButton * move_vertex_mode_button_{nullptr};
  QPushButton * delete_vertex_button_{nullptr};
  QPushButton * delete_edge_button_{nullptr};
  QComboBox * kind_combo_{nullptr};
  QComboBox * id_combo_{nullptr};
  QCheckBox * batch_pick_enabled_{nullptr};
  QPushButton * batch_start_button_{nullptr};
  QPushButton * batch_end_button_{nullptr};
  QPushButton * batch_clear_button_{nullptr};
  QPushButton * batch_apply_button_{nullptr};
  QComboBox * batch_field_combo_{nullptr};
  QLabel * batch_status_label_{nullptr};
  QStackedWidget * editor_stack_{nullptr};
  QLabel * status_label_{nullptr};

  QLabel * vertex_id_label_{nullptr};
  QDoubleSpinBox * vertex_x_{nullptr};
  QDoubleSpinBox * vertex_y_{nullptr};
  QDoubleSpinBox * vertex_z_{nullptr};
  QDoubleSpinBox * vertex_roll_{nullptr};
  QDoubleSpinBox * vertex_pitch_{nullptr};
  QDoubleSpinBox * vertex_yaw_{nullptr};
  QSpinBox * vertex_type_{nullptr};
  QSpinBox * vertex_type_id_{nullptr};
  QCheckBox * vertex_corner_{nullptr};
  QCheckBox * vertex_slope_{nullptr};
  QCheckBox * vertex_junction_{nullptr};
  QSpinBox * vertex_charging_mode_{nullptr};
  QDoubleSpinBox * vertex_acc_{nullptr};
  QCheckBox * vertex_turnable_{nullptr};
  QCheckBox * vertex_align_yaw_{nullptr};
  QCheckBox * vertex_must_pass_{nullptr};
  QDoubleSpinBox * vertex_pass_radius_{nullptr};
  QLineEdit * vertex_pcd_{nullptr};

  QLabel * edge_id_label_{nullptr};
  QLabel * edge_endpoints_label_{nullptr};
  QLabel * edge_weight_label_{nullptr};
  QLabel * edge_source_label_{nullptr};
  QComboBox * edge_obstacle_mode_{nullptr};
  QComboBox * edge_controller_mode_{nullptr};
  QComboBox * edge_travel_mode_{nullptr};
  QCheckBox * edge_rotation_allowed_{nullptr};
  QDoubleSpinBox * edge_linear_speed_{nullptr};
  QDoubleSpinBox * edge_angular_speed_{nullptr};
  QDoubleSpinBox * edge_height_offset_{nullptr};
  QDoubleSpinBox * edge_heading_angle_{nullptr};
  QLineEdit * edge_obstacle_box_{nullptr};
  QLineEdit * edge_grid_map_{nullptr};
};

}  // namespace route3d_topology_editor

#endif  // ROUTE3D_TOPOLOGY_EDITOR__TOPOLOGY_EDITOR_PANEL_HPP_
