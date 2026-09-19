#include "plan_map_rviz/goal3d_tool.h"

#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>

namespace plan_map_rviz
{

Goal3DTool::Goal3DTool()
{
  shortcut_key_ = 'g';
  topic_property_ = new rviz::StringProperty(
      "Topic",
      "/move_base_simple/goal",
      "The topic on which to publish 3D navigation goals.",
      getPropertyContainer(),
      &Goal3DTool::updateTopic,
      this);
}

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName("3D Nav Goal");
  setArrowColor(1.0f, 0.15f, 0.85f);
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
}

void Goal3DTool::onPoseSet(double x, double y, double z, double yaw)
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  ROS_INFO("3D Nav Goal: (%.2f, %.2f, %.2f) yaw=%.2f -> %s",
           x, y, z, yaw, topic_property_->getStdString().c_str());
  pub_.publish(msg);
}

}  // namespace plan_map_rviz

PLUGINLIB_EXPORT_CLASS(plan_map_rviz::Goal3DTool, rviz::Tool)
