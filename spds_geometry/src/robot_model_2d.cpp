#include <spds_geometry/robot_model_2d.h>

using namespace spds_geometry;

RobotModel2dWithState::RobotModel2dWithState(const RobotModel2dInterface &m) : model(m) {

}

void
RobotModel2dWithState::update(const spds_generic::Pose2d &p, const spds_generic::RobotInternalState2d &is)
{
  // Get the polygon from the model (given in robot coordinate frame), rotate it accoding from the states.
  const Polygon& poly = model.getBoundingRegion(is);
  posePolygon = Polygon(poly); // Overwrite the poly with the current model configuration...
  movePoint2dContainer(posePolygon, p); // ... and move it to the right pose.
  robotPose = p;
}

