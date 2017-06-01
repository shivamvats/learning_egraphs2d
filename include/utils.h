#ifndef UTILS_H
#define UTILS_H

#include <boost/filesystem.hpp>
#include <fstream>
#include <string>

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/occupancy_grid.h>
#include <smpl/robot_model.h>
#include <smpl/types.h>

#include <leatherman/print.h>
#include <leatherman/viz.h>

namespace smpl = sbpl::motion;

bool writePath(const moveit_msgs::RobotState &,
               const moveit_msgs::RobotTrajectory &traj,
               smpl::RobotModel *robotModel, std::string plan_output_dir);

void convertJointVariablePathToJointTrajectory(
    const std::vector<smpl::RobotState> &path,
    trajectory_msgs::JointTrajectory &traj, smpl::RobotModel *robotModel,
    std::string planning_frame);

void PrintGrid(std::ostream &, sbpl::OccupancyGrid &);
void PrintActionSpace(const smpl::ManipLatticeActionSpace &);

#endif
