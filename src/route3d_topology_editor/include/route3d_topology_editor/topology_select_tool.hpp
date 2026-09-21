#ifndef ROUTE3D_TOPOLOGY_EDITOR__TOPOLOGY_SELECT_TOOL_HPP_
#define ROUTE3D_TOPOLOGY_EDITOR__TOPOLOGY_SELECT_TOOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace route3d_topology_editor
{

class TopologySelectTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  TopologySelectTool();
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
};

}  // namespace route3d_topology_editor

#endif  // ROUTE3D_TOPOLOGY_EDITOR__TOPOLOGY_SELECT_TOOL_HPP_
