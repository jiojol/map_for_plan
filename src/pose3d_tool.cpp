#include "plan_map_rviz/pose3d_tool.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <OgrePlane.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <QKeyEvent>

#include <rviz/display_context.h>
#include <rviz/geometry.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/viewport_mouse_event.h>

namespace plan_map_rviz
{
namespace
{
const double kMinYawDistance = 0.05;
const double kWheelStep = 0.10;
const double kWheelStepFine = 0.02;
}

Pose3DTool::Pose3DTool()
  : state_(Position)
  , pos_(Ogre::Vector3::ZERO)
  , yaw_(0.0)
  , last_z_(0.0)
  , pose_arrow_(nullptr)
  , height_arrow_(nullptr)
{
  access_all_keys_ = true;
}

Pose3DTool::~Pose3DTool()
{
  delete pose_arrow_;
  delete height_arrow_;
}

void Pose3DTool::onInitialize()
{
  pose_arrow_ = new rviz::Arrow(scene_manager_, nullptr, 1.0f, 0.12f, 0.28f, 0.22f);
  height_arrow_ = new rviz::Arrow(scene_manager_, nullptr, 0.8f, 0.08f, 0.18f, 0.14f);
  pose_arrow_->getSceneNode()->setVisible(false);
  height_arrow_->getSceneNode()->setVisible(false);
  setArrowColor(1.0f, 0.15f, 0.85f);
}

void Pose3DTool::setArrowColor(float r, float g, float b)
{
  if (pose_arrow_)
  {
    pose_arrow_->setColor(r, g, b, 1.0f);
  }
  if (height_arrow_)
  {
    height_arrow_->setColor(0.2f, 0.9f, 0.3f, 1.0f);
  }
}

void Pose3DTool::activate()
{
  reset();
  setStatus("3D 工具：按下取点（含高度），拖动设朝向，滚轮或 ↑↓ 改高度，松开发布。");
}

void Pose3DTool::deactivate()
{
  reset();
}

void Pose3DTool::reset()
{
  state_ = Position;
  yaw_ = 0.0;
  if (pose_arrow_)
  {
    pose_arrow_->getSceneNode()->setVisible(false);
  }
  if (height_arrow_)
  {
    height_arrow_->getSceneNode()->setVisible(false);
  }
}

bool Pose3DTool::pickPosition(rviz::ViewportMouseEvent& event, Ogre::Vector3& pos)
{
  if (context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, pos))
  {
    return true;
  }

  // Tomogram cells are rendered as separated points, so a one-pixel depth
  // query often lands in the gap between two cells.  Search a small screen
  // patch and take the valid depth sample nearest to the cursor.
  constexpr int kPickRadius = 8;
  const int viewport_width = static_cast<int>(event.viewport->getActualWidth());
  const int viewport_height = static_cast<int>(event.viewport->getActualHeight());
  if (viewport_width <= 0 || viewport_height <= 0)
  {
    return false;
  }
  const int x0 = std::max(0, event.x - kPickRadius);
  const int y0 = std::max(0, event.y - kPickRadius);
  const int x1 = std::min(viewport_width - 1, event.x + kPickRadius);
  const int y1 = std::min(viewport_height - 1, event.y + kPickRadius);
  const unsigned width = static_cast<unsigned>(x1 - x0 + 1);
  const unsigned height = static_cast<unsigned>(y1 - y0 + 1);

  std::vector<Ogre::Vector3> points;
  if (!context_->getSelectionManager()->get3DPatch(
          event.viewport, x0, y0, width, height, false, points))
  {
    return false;
  }

  double best_distance_sq = std::numeric_limits<double>::infinity();
  bool found = false;
  for (std::size_t index = 0; index < points.size(); ++index)
  {
    const Ogre::Vector3& candidate = points[index];
    if (!std::isfinite(candidate.x) || !std::isfinite(candidate.y) ||
        !std::isfinite(candidate.z))
    {
      continue;
    }
    const int px = x0 + static_cast<int>(index % width);
    const int py = y0 + static_cast<int>(index / width);
    const double dx = static_cast<double>(px - event.x);
    const double dy = static_cast<double>(py - event.y);
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < best_distance_sq)
    {
      best_distance_sq = distance_sq;
      pos = candidate;
      found = true;
    }
  }
  return found;
}

bool Pose3DTool::yawFromMouse(rviz::ViewportMouseEvent& event, double& yaw) const
{
  Ogre::Plane plane(Ogre::Vector3::UNIT_Z, static_cast<Ogre::Real>(pos_.z));
  Ogre::Vector3 cur;
  if (!rviz::getPointOnPlaneFromWindowXY(event.viewport, plane, event.x, event.y, cur))
  {
    return false;
  }
  const double dx = static_cast<double>(cur.x - pos_.x);
  const double dy = static_cast<double>(cur.y - pos_.y);
  if ((dx * dx) + (dy * dy) < (kMinYawDistance * kMinYawDistance))
  {
    return false;
  }
  yaw = std::atan2(dy, dx);
  return true;
}

void Pose3DTool::applyWheelHeight(rviz::ViewportMouseEvent& event)
{
  if (event.wheel_delta == 0)
  {
    return;
  }
  const double step = event.shift() ? kWheelStepFine : kWheelStep;
  pos_.z += step * (static_cast<double>(event.wheel_delta) / 120.0);
  last_z_ = pos_.z;
}

void Pose3DTool::updateArrows()
{
  pose_arrow_->setPosition(pos_);
  pose_arrow_->setDirection(Ogre::Vector3(std::cos(yaw_), std::sin(yaw_), 0.0));
  pose_arrow_->getSceneNode()->setVisible(true);

  const double shaft = std::max(0.35, static_cast<double>(std::abs(pos_.z)));
  height_arrow_->set(static_cast<float>(shaft), 0.08f, 0.18f, 0.14f);
  height_arrow_->setPosition(Ogre::Vector3(pos_.x, pos_.y, 0.0));
  height_arrow_->setDirection(pos_.z >= 0.0 ? Ogre::Vector3::UNIT_Z : -Ogre::Vector3::UNIT_Z);
  height_arrow_->getSceneNode()->setVisible(true);
}

void Pose3DTool::updateStatus()
{
  setStatus(QString("x=%1  y=%2  z=%3  yaw=%4°。拖动朝向，滚轮或 ↑↓ 改高度，松开发布。")
                .arg(pos_.x, 0, 'f', 2)
                .arg(pos_.y, 0, 'f', 2)
                .arg(pos_.z, 0, 'f', 2)
                .arg(yaw_ * 180.0 / 3.14159265358979323846, 0, 'f', 1));
}

int Pose3DTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = Render;

  if (event.rightDown())
  {
    reset();
    setStatus("已取消 3D 工具。");
    return flags;
  }

  if (event.leftDown() && state_ == Position)
  {
    Ogre::Vector3 hit;
    if (!pickPosition(event, hit))
    {
      setStatus("未命中可通行体素，请放大后点击绿色规划地图。");
      return flags;
    }
    pos_ = hit;
    last_z_ = pos_.z;
    yaw_ = 0.0;
    state_ = Active;
    updateArrows();
    updateStatus();
    return flags;
  }

  if (state_ != Active)
  {
    return flags;
  }

  if (event.type == QEvent::Wheel)
  {
    applyWheelHeight(event);
    updateArrows();
    updateStatus();
    return flags;
  }

  if (event.type == QEvent::MouseMove && event.left())
  {
    double yaw = yaw_;
    if (yawFromMouse(event, yaw))
    {
      yaw_ = yaw;
    }
    updateArrows();
    updateStatus();
    return flags;
  }

  if (event.leftUp())
  {
    double yaw = yaw_;
    if (yawFromMouse(event, yaw))
    {
      yaw_ = yaw;
    }
    onPoseSet(pos_.x, pos_.y, pos_.z, yaw_);
    reset();
    flags |= Finished;
  }

  return flags;
}

int Pose3DTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel*)
{
  if (event->key() == Qt::Key_Escape)
  {
    reset();
    setStatus("已取消 3D 工具。");
    return Render;
  }

  if (state_ == Active)
  {
    const bool fine = event->modifiers() & Qt::ShiftModifier;
    const double step = fine ? kWheelStepFine : kWheelStep;
    if (event->key() == Qt::Key_Up || event->key() == Qt::Key_Plus ||
        event->key() == Qt::Key_Equal || event->key() == Qt::Key_PageUp)
    {
      pos_.z += step;
      last_z_ = pos_.z;
      updateArrows();
      updateStatus();
      return Render;
    }
    if (event->key() == Qt::Key_Down || event->key() == Qt::Key_Minus ||
        event->key() == Qt::Key_PageDown)
    {
      pos_.z -= step;
      last_z_ = pos_.z;
      updateArrows();
      updateStatus();
      return Render;
    }
  }
  return 0;
}

}  // namespace plan_map_rviz
