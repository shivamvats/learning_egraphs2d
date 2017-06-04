#ifndef UTILS_H
#define UTILS_H

#include <boost/filesystem.hpp>
#include <fstream>
#include <string>
#include <math.h>
#include <algorithm>

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/occupancy_grid.h>
#include <smpl/robot_model.h>
#include <smpl/types.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <leatherman/print.h>
#include <leatherman/viz.h>

#define PI 3.14

namespace smpl = sbpl::motion;
using namespace Eigen;

int getRandomInt(int min, int max);
float getRandomFloat(float min, float max);
std::vector<Vector3d> samplePointsOnCircle(Vector3d, double radius, int num);

bool writePath(const moveit_msgs::RobotState &,
               const moveit_msgs::RobotTrajectory &traj,
               smpl::RobotModel *robotModel, std::string plan_output_dir);

void convertJointVariablePathToJointTrajectory(
    const std::vector<smpl::RobotState> &path,
    trajectory_msgs::JointTrajectory &traj, smpl::RobotModel *robotModel,
    std::string planning_frame);

void PrintGrid(std::ostream &, sbpl::OccupancyGrid &);
void PrintActionSpace(const smpl::ManipLatticeActionSpace &);
visualization_msgs::Marker getPointVisualization(geometry_msgs::Point,
                                                 std::string, std::string);
visualization_msgs::Marker
    getLineVisualization(std::vector<geometry_msgs::Point>, std::string,
                         std::string);
void translatePoint(Vector3d &point, Vector3d origin);

#endif
