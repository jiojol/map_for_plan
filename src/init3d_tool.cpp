#include "plan_map_rviz/init3d_tool.h"

#include <cmath>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rviz/display_context.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>

namespace plan_map_rviz
{

Init3DTool::Init3DTool()
{
  shortcut_key_ = 'p';
  topic_property_ = new rviz::StringProperty(
      "Topic",
      "/initialpose",
      "The topic on which to publish the 3D initial pose.",
      getPropertyContainer(),
      &Init3DTool::updateTopic,
      this);
  std_dev_x_ = new rviz::FloatProperty(
      "X std deviation", 0.5, "X standard deviation for the initial pose covariance.",
      getPropertyContainer());
  std_dev_y_ = new rviz::FloatProperty(
      "Y std deviation", 0.5, "Y standard deviation for the initial pose covariance.",
      getPropertyContainer());
  std_dev_z_ = new rviz::FloatProperty(
      "Z std deviation", 0.5, "Z standard deviation for the initial pose covariance.",
      getPropertyContainer());
  std_dev_theta_ = new rviz::FloatProperty(
      "Theta std deviation", 0.2618, "Yaw standard deviation for the initial pose covariance.",
      getPropertyContainer());
  std_dev_x_->setMin(0.0);
  std_dev_y_->setMin(0.0);
  std_dev_z_->setMin(0.0);
  std_dev_theta_->setMin(0.0);
}

void Init3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName("3D Pose Estimate");
  setArrowColor(0.15f, 0.85f, 0.25f);
  updateTopic();
}

void Init3DTool::updateTopic()
{
  pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_property_->getStdString(), 1);
}

void Init3DTool::onPoseSet(double x, double y, double z, double yaw)
{
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.header.stamp = ros::Time::now();
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  const double sx = std_dev_x_->getFloat();
  const double sy = std_dev_y_->getFloat();
  const double sz = std_dev_z_->getFloat();
  const double st = std_dev_theta_->getFloat();
  msg.pose.covariance[6 * 0 + 0] = sx * sx;
  msg.pose.covariance[6 * 1 + 1] = sy * sy;
  msg.pose.covariance[6 * 2 + 2] = sz * sz;
  msg.pose.covariance[6 * 5 + 5] = st * st;

  ROS_INFO("3D Pose Estimate: (%.2f, %.2f, %.2f) yaw=%.2f -> %s",
           x, y, z, yaw, topic_property_->getStdString().c_str());
  pub_.publish(msg);
}

}  // namespace plan_map_rviz

PLUGINLIB_EXPORT_CLASS(plan_map_rviz::Init3DTool, rviz::Tool)
