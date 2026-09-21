#include "route3d_topology_editor/topology_select_tool.hpp"

#include <QCursor>

#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreVector.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

namespace route3d_topology_editor
{

TopologySelectTool::TopologySelectTool()
{
  shortcut_key_ = 's';
}

void TopologySelectTool::onInitialize()
{
  const auto abstraction = context_->getRosNodeAbstraction().lock();
  if (!abstraction) {
    return;
  }
  node_ = abstraction->get_raw_node();
  publisher_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
    "/route3d_topology_editor/select_point", rclcpp::QoS(10).reliable());
  setName("选择拓扑");
  setDescription("单击图上的顶点或边，在 Topology Editor 面板中选中最近项");
}

void TopologySelectTool::activate()
{
  setCursor(Qt::CrossCursor);
}

void TopologySelectTool::deactivate()
{
}

int TopologySelectTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!event.leftUp() || !publisher_ || !node_) {
    return Render;
  }
  Ogre::Vector3 picked;
  if (!context_->getViewPicker()->get3DPoint(
      event.panel, event.x, event.y, picked))
  {
    const auto * camera = event.panel->getViewController()->getCamera();
    const float width = static_cast<float>(event.panel->width());
    const float height = static_cast<float>(event.panel->height());
    if (!camera || width <= 0.0F || height <= 0.0F) {
      return Render;
    }
    const Ogre::Ray ray = camera->getCameraToViewportRay(event.x / width, event.y / height);
    const auto intersection = ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z, 0.0F));
    if (!intersection.first) {
      return Render;
    }
    picked = ray.getPoint(intersection.second);
  }
  geometry_msgs::msg::PointStamped message;
  message.header.stamp = node_->now();
  message.header.frame_id = context_->getFixedFrame().toStdString();
  message.point.x = picked.x;
  message.point.y = picked.y;
  message.point.z = picked.z;
  publisher_->publish(message);
  return Render;
}

}  // namespace route3d_topology_editor

PLUGINLIB_EXPORT_CLASS(route3d_topology_editor::TopologySelectTool, rviz_common::Tool)
