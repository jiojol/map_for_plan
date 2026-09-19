#ifndef PLAN_MAP_RVIZ_POSE3D_TOOL_H
#define PLAN_MAP_RVIZ_POSE3D_TOOL_H

#include <OgreVector3.h>
#include <rviz/tool.h>

namespace rviz
{
class Arrow;
class DisplayContext;
}

namespace plan_map_rviz
{

class Pose3DTool : public rviz::Tool
{
public:
  Pose3DTool();
  ~Pose3DTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double yaw) = 0;
  void setArrowColor(float r, float g, float b);

private:
  void reset();
  bool pickPosition(rviz::ViewportMouseEvent& event, Ogre::Vector3& pos);
  bool yawFromMouse(rviz::ViewportMouseEvent& event, double& yaw) const;
  void applyWheelHeight(rviz::ViewportMouseEvent& event);
  void updateArrows();
  void updateStatus();

  enum State
  {
    Position,
    Active
  };

  State state_;
  Ogre::Vector3 pos_;
  double yaw_;
  double last_z_;
  rviz::Arrow* pose_arrow_;
  rviz::Arrow* height_arrow_;
};

}  // namespace plan_map_rviz

#endif
