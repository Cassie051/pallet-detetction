#pragma once

#include <spds_generic/types.h>
#include <spds_geometry/polygon.h>
#include <spds_msgs/Path.h>
#include <spds_msgs/Trajectory.h>
#include <spds_msgs/ControllerTrajectoryChunk.h>
#include <spds_msgs/ControllerTrajectoryStep.h>
#include <spds_msgs/DeltaT.h>
#include <spds_msgs/DeltaTVec.h>
#include <spds_msgs/PolygonConstraint.h>
#include <spds_msgs/RobotConstraints.h>
#include <spds_msgs/CoordinatorTimeVec.h>
#include <spds_msgs/ParkingPolygons.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_datatypes.h>


namespace spds_conversions
{

  spds_generic::Pose2d createPose2dFromMsg(const geometry_msgs::Pose& pose)
    {
      spds_generic::Pose2d p;
      p(0) = pose.position.x;
      p(1) = pose.position.y;
      p(2) = tf::getYaw(pose.orientation);
      return p;
    }

  geometry_msgs::Pose createMsgFromPose2d(const spds_generic::Pose2d& pose)
    {
      geometry_msgs::Pose p;
      p.position.x = pose(0);
      p.position.y = pose(1);
      p.position.z = 0.;
      p.orientation = tf::createQuaternionMsgFromYaw(pose(2));
      return p;
    }

  spds_generic::Pose2dCov createPose2dCovFromMsg(const geometry_msgs::PoseWithCovariance &posecov) 
  {
    spds_generic::Pose2dCov p;
    p.mean = createPose2dFromMsg(posecov.pose);
    p.cov(0,0) = posecov.covariance[0];
    p.cov(0,1) = posecov.covariance[1];
    p.cov(0,2) = posecov.covariance[5];
    p.cov(1,0) = posecov.covariance[6];
    p.cov(1,1) = posecov.covariance[7];
    p.cov(1,2) = posecov.covariance[11];
    p.cov(2,0) = posecov.covariance[30];
    p.cov(2,1) = posecov.covariance[31];
    p.cov(2,2) = posecov.covariance[35];
    return p;
  }

  geometry_msgs::PoseWithCovariance createMsgFromPose2dCov(const spds_generic::Pose2dCov &p) 
  {
    geometry_msgs::PoseWithCovariance posecov;
    posecov.pose = createMsgFromPose2d(p.mean);
    posecov.covariance[0] = p.cov(0,0); 
    posecov.covariance[1] = p.cov(0,1);
    posecov.covariance[5] = p.cov(0,2);
    posecov.covariance[6] = p.cov(1,0);
    posecov.covariance[7] = p.cov(1,1);
    posecov.covariance[11] = p.cov(1,2);
    posecov.covariance[30] = p.cov(2,0);
    posecov.covariance[31] = p.cov(2,1);
    posecov.covariance[35] = p.cov(2,2);
    return posecov;
  }

  spds_generic::Pose2d createPose2dFromControllerStateMsg(const spds_msgs::ControllerState& state)
    {
      spds_generic::Pose2d p;
      p(0) = state.position_x;
      p(1) = state.position_y;
      p(2) = state.orientation_angle;
      return p;
    }

  spds_generic::State2d createState2dFromControllerStateMsg(const spds_msgs::ControllerState& state)
    {
      spds_generic::Pose2d p;
      p(0) = state.position_x;
      p(1) = state.position_y;
      p(2) = state.orientation_angle;
      spds_generic::State2d s;
      s.setPose2d(p);
      s.setSteeringAngle(state.steering_angle);
      return s;
    }

  spds_generic::State2d createState2dFromPoseSteeringMsg(const spds_msgs::PoseSteering &ps)
    {
      spds_generic::State2d s;
      s.pose = createPose2dFromMsg(ps.pose);
      s.steeringAngle = ps.steering;
      return s;
    }

  spds_msgs::PoseSteering createPoseSteeringMsgFromState2d(const spds_generic::State2d &state)
    {
      spds_msgs::PoseSteering ps;
      ps.pose = createMsgFromPose2d(state.getPose2d());
      ps.steering = state.steeringAngle;
      return ps;
    }

  spds_msgs::Path createPathMsgFromPathInterface(const spds_generic::PathInterface &path)
{
  spds_msgs::Path p;
  for (size_t i = 0; i < path.sizePath(); i++)
    {
      spds_msgs::PoseSteering ps;
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(path.getPose2d(i)(2));
      ps.pose.position.x = path.getPose2d(i)(0);
      ps.pose.position.y = path.getPose2d(i)(1);
      ps.pose.position.z = 0.;
      ps.steering = path.getSteeringAngle(i);
      p.path.push_back(ps);
    }
  return p;
}

  spds_msgs::Path createPathMsgFromPathAndState2dInterface(const spds_generic::PathInterface &path,
							    const spds_generic::State2dInterface &start,
							    const spds_generic::State2dInterface &goal)
{
  spds_msgs::Path p = createPathMsgFromPathInterface(path);
  // Fill in the targets.
  p.target_start = createPoseSteeringMsgFromState2d(start);
  p.target_goal = createPoseSteeringMsgFromState2d(goal);
  return p;
}



  spds_generic::Path createPathFromPathMsg(const spds_msgs::Path &path)
    {
      spds_generic::Path p;
      for (unsigned int i = 0; i < path.path.size(); i++)
	{
	  p.addPathPoint(createPose2dFromMsg(path.path[i].pose), path.path[i].steering);
	}
      return p;
    }

  spds_generic::Path createPathFromPathMsgUsingTargetsAsFirstLast(const spds_msgs::Path &path) {
      spds_generic::Path p;
      p.addPathPoint(createPose2dFromMsg(path.target_start.pose), path.target_start.steering);
      for (unsigned int i = 1; i < path.path.size()-1; i++) {
	p.addPathPoint(createPose2dFromMsg(path.path[i].pose), path.path[i].steering);
      }
      p.addPathPoint(createPose2dFromMsg(path.target_goal.pose), path.target_goal.steering);
      return p;
  }

  spds_msgs::Trajectory createTrajectoryMsgFromTrajectoryInterface(const spds_generic::TrajectoryInterface &traj)
    {
      spds_msgs::Trajectory t;
      for (unsigned int i = 0; i < traj.sizeTrajectory(); i++)
	{
	  spds_msgs::TrajectoryPoint tp;
	  tp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(traj.getPose2d(i)(2));
	  tp.pose.pose.position.x = traj.getPose2d(i)(0);
	  tp.pose.pose.position.y = traj.getPose2d(i)(1);
	  tp.pose.pose.position.z = 0.;
	  tp.pose.steering = traj.getSteeringAngle(i);

	  tp.velocity.drive = traj.getDriveVel(i);
	  tp.velocity.steering = traj.getSteeringVel(i);
	  
	  t.trajectory.push_back(tp);
	}
      return t;
    }

  spds_generic::Trajectory createTrajectoryFromTrajectoryMsg(const spds_msgs::Trajectory &traj)
    {
      spds_generic::Trajectory t;
      for (unsigned int i = 0; i < traj.trajectory.size(); i++)
	{
	  t.addTrajectoryPoint(createPose2dFromMsg(traj.trajectory[i].pose.pose), traj.trajectory[i].pose.steering, traj.trajectory[i].velocity.drive, traj.trajectory[i].velocity.steering);
	}
      return t;
    }

  spds_msgs::ControllerTrajectoryChunk createControllerTrajectoryChunkFromTrajectoryInterface(const spds_generic::TrajectoryInterface &traj)
    {
      spds_msgs::ControllerTrajectoryChunk c;
      for (unsigned int i = 0; i < traj.sizeTrajectory(); i++)
	{
	  //c.constaints - TODO
	  spds_msgs::ControllerTrajectoryStep s;
	  s.state.position_x = traj.getPose2d(i)(0);
	  s.state.position_y = traj.getPose2d(i)(1);
	  s.state.orientation_angle = traj.getPose2d(i)(2);
	  s.state.steering_angle = traj.getSteeringAngle(i);
	  
	  s.velocities.tangential = traj.getDriveVel(i);
	  s.velocities.steering = traj.getSteeringVel(i);
	  
	  s.mode = spds_msgs::ControllerTrajectoryStep::MODE_1;

	  c.steps.push_back(s);
	}
      return c;
    }

  spds_msgs::DeltaT createDeltaTMsgFromDeltaTInterface(const spds_generic::DeltaTInterface &delta)
    {
      spds_msgs::DeltaT dt;
      for (unsigned int i = 0; i < delta.sizeDeltaTVec(); i++)
	{
	  dt.dt.push_back(delta.getDeltaT(i));
	}
      return dt;
    }
  

  spds_msgs::PolygonConstraint createPolygonConstraintMsgFromConvexPolygon(const spds_geometry::Polygon &polygon)
    {
      spds_msgs::PolygonConstraint poly;
      Eigen::MatrixXd A;
      Eigen::VectorXd b; 		 
      double *dt;
  
      polygon.getMatrixForm(A,b);

      //      std::cout << "A: " << A << std::endl;
      //      std::cout << "b: " << b << std::endl;

      dt = A.col(0).data();
      poly.A0 = std::vector<double>(dt,dt+A.rows());
      dt = A.col(1).data();
      poly.A1 = std::vector<double>(dt,dt+A.rows());
      dt = b.data();
      poly.b = std::vector<double>(dt,dt+b.rows());
      poly.theta_min = -M_PI; // TODO
      poly.theta_max = M_PI;  // TODO
      Eigen::Vector2d center = getCenter(polygon);
      poly.feasible_x = center(0);
      poly.feasible_y = center(1);
      return poly;
    }

  spds_geometry::Polygon createConvexPolygonFromPolygonConstraintMsg(const spds_msgs::PolygonConstraint &msg) {
    std::vector<double> A0 = msg.A0;
    std::vector<double> A1 = msg.A1;
    std::vector<double> b  = msg.b;

    assert(A0.size() == A1.size() && A0.size() == b.size());
    //assert(A0.size() > 1); // Possible to handle empty constraints?
    // Find intersecting points - here we assume that the lines comes in order.
    spds_generic::Point2dVec pts;
    for (size_t i = 0; i < A0.size(); i++)
      {
	Eigen::Vector2d inter;
	if (i == A0.size() -1)
	  inter = spds_geometry::getIntersection(A0[i], A1[i], A0[0], A1[0], b[i], b[0]);
	else
	  inter = spds_geometry::getIntersection(A0[i], A1[i], A0[i+1], A1[i+1], b[i], b[i+1]);
	pts.push_back(inter);
      }
    return spds_geometry::Polygon(pts);
  }

  spds_msgs::RobotConstraints createRobotConstraintsMsgFromConvexPolygons(const spds_geometry::Polygons &polygons, std::vector<unsigned int> &subSampledIdx)
    {
      spds_msgs::RobotConstraints constraints;
      for(unsigned int i = 0; i< polygons.size(); i++) {
	spds_msgs::PolygonConstraint poly = createPolygonConstraintMsgFromConvexPolygon(polygons[i]);
	poly.constraint_id = i;
	constraints.constraints.push_back(poly);
      }
      for(unsigned int i = 0; i < subSampledIdx.size(); i++) {
	constraints.points.push_back(subSampledIdx[i]);
      }
      return constraints;
    }

  spds_msgs::RobotConstraints createRobotConstraintsMsgFromConvexPolygons(const spds_geometry::Polygons &polygons)
    {
      spds_msgs::RobotConstraints constraints;
      for(unsigned int i = 0; i< polygons.size(); i++) {
	spds_msgs::PolygonConstraint poly = createPolygonConstraintMsgFromConvexPolygon(polygons[i]);
	poly.constraint_id = i;
	constraints.constraints.push_back(poly);
      }
      return constraints;
    }

spds_msgs::RobotConstraints createRobotConstraintsMsgFromConvexPolygons(const std::vector<spds_geometry::Polygon> &polygons)
    {
      spds_msgs::RobotConstraints constraints;
      for(unsigned int i = 0; i< polygons.size(); i++) {
	spds_msgs::PolygonConstraint poly = createPolygonConstraintMsgFromConvexPolygon(polygons[i]);
	poly.constraint_id = i;
	constraints.constraints.push_back(poly);
      }
      return constraints;
    }


  std::vector<spds_geometry::Polygon> createConvexPolygonsFromRobotConstraintsMsg(const spds_msgs::RobotConstraints &msg) {
    std::vector<spds_geometry::Polygon> ret;
    for (unsigned int i = 0; i < msg.constraints.size(); i++) {
      ret.push_back(createConvexPolygonFromPolygonConstraintMsg(msg.constraints[i]));
    }
    return ret;
  }

  std::vector<spds_geometry::Polygon> createConvexOuterPolygonsFromRobotConstraintsMsg(const spds_msgs::RobotConstraints &msg) {
    std::vector<spds_geometry::Polygon> ret;
    for (unsigned int i = 0; i < msg.constraints_outer.size(); i++) {
      ret.push_back(createConvexPolygonFromPolygonConstraintMsg(msg.constraints_outer[i]));
    }
    return ret;
  }

std::vector<double> getDoubleVecFromDeltaTMsg(const spds_msgs::DeltaT &msg) {
  std::vector<double> times(msg.dt.size());
  for (size_t i = 0; i < msg.dt.size(); i++) {
    times[i] = msg.dt[i];
  }
  return times;
}

  std::vector<double> getDoubleVecFromCoordinatorTimeMsg(const spds_msgs::CoordinatorTime &msg) {
    std::vector<double> times(msg.t.size());
    for (unsigned int i = 0; i < msg.t.size(); i++) {
      times[i] = msg.t[i];
    }
    return times;
  }

spds_generic::CoordinatedTimes getCoordinatedTimesFromCoordinatorTimeMsg(const spds_msgs::CoordinatorTime &msg) {
  spds_generic::CoordinatedTimes cts(getDoubleVecFromCoordinatorTimeMsg(msg));
  return cts;
}

  std::vector<std::vector<double> > getDoubleVecsFromCoordinatorTimeVecMsg(const spds_msgs::CoordinatorTimeVec &msg) {
    std::vector<std::vector<double> > times(msg.ts.size());
    for (unsigned int i = 0; i < msg.ts.size(); i++) {
      times[i] = getDoubleVecFromCoordinatorTimeMsg(msg.ts[i]);
    }
    return times;
  }

  spds_msgs::CoordinatorTime getCoordinatorTimeFromDoubleVec(const std::vector<double> &vec) {
    spds_msgs::CoordinatorTime msg;
    for (unsigned int i = 0; i < vec.size(); i++) {
      msg.t.push_back(vec[i]);
    }
    return msg;
  }

  std::map<int, std::vector<spds_geometry::Polygon> > getRobotIDPolygonMapFromParkingPolygonsMsg(const spds_msgs::ParkingPolygons &msg) {
    std::map<int, std::vector<spds_geometry::Polygon> > ret;
    for (unsigned int i = 0; i < msg.robot_ids.size(); i++) {
      std::vector<spds_geometry::Polygon> polys;
      polys.push_back(createConvexPolygonFromPolygonConstraintMsg(msg.parking_polygons_intensity1[i]));
      polys.push_back(createConvexPolygonFromPolygonConstraintMsg(msg.parking_polygons_intensity2[i]));
      polys.push_back(createConvexPolygonFromPolygonConstraintMsg(msg.parking_polygons_intensity3[i]));
      ret.insert(std::pair<int, std::vector<spds_geometry::Polygon> >(msg.robot_ids[i], polys));
    }
    return ret;
  }

} // namespace
