#include "utils.h"

#include <chrono>
#include <smpl/time.h>
#include <utility>

bool writePath(const moveit_msgs::RobotState &ref,
               const moveit_msgs::RobotTrajectory &traj,
               smpl::RobotModel *robotModel, std::string plan_output_dir) {
    boost::filesystem::path p(plan_output_dir);

    try {
        if (!boost::filesystem::exists(p)) {
            ROS_INFO("Create plan output directory %s", p.native().c_str());
            boost::filesystem::create_directory(p);
        }

        if (!boost::filesystem::is_directory(p)) {
            ROS_ERROR("Failed to log path. %s is not a directory",
                      plan_output_dir.c_str());
            return false;
        }
    } catch (const boost::filesystem::filesystem_error &ex) {
        ROS_ERROR("Failed to create plan output directory %s",
                  p.native().c_str());
        return false;
    }

    std::stringstream ss_filename;
    auto now = std::chrono::high_resolution_clock::now();

    ss_filename << "path_" << now.time_since_epoch().count();
    p /= ss_filename.str();

    std::ofstream ofs(p.native());
    if (!ofs.is_open()) {
        return false;
    }

    ROS_INFO("Log path to %s", p.native().c_str());

    // write header
    for (size_t vidx = 0; vidx < robotModel->jointVariableCount(); ++vidx) {
        const std::string &var_name = robotModel->getPlanningJoints()[vidx];
        ofs << var_name; // TODO: sanitize variable name for csv?
        if (vidx != robotModel->jointVariableCount() - 1) {
            ofs << ',';
        }
    }
    ofs << '\n';

    const size_t wp_count =
        std::max(traj.joint_trajectory.points.size(),
                 traj.multi_dof_joint_trajectory.points.size());
    for (size_t widx = 0; widx < wp_count; ++widx) {
        // fill the complete robot state
        moveit_msgs::RobotState state = ref;

        if (widx < traj.joint_trajectory.points.size()) {
            const trajectory_msgs::JointTrajectoryPoint &wp =
                traj.joint_trajectory.points[widx];
            const size_t joint_count = traj.joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                const std::string &joint_name =
                    traj.joint_trajectory.joint_names[jidx];
                double vp = wp.positions[jidx];
                auto it = std::find(state.joint_state.name.begin(),
                                    state.joint_state.name.end(), joint_name);
                if (it != state.joint_state.name.end()) {
                    size_t tvidx =
                        std::distance(state.joint_state.name.begin(), it);
                    state.joint_state.position[tvidx] = vp;
                }
            }
        }
        if (widx < traj.multi_dof_joint_trajectory.points.size()) {
            const trajectory_msgs::MultiDOFJointTrajectoryPoint &wp =
                traj.multi_dof_joint_trajectory.points[widx];
            const size_t joint_count =
                traj.multi_dof_joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                const std::string &joint_name =
                    traj.multi_dof_joint_trajectory.joint_names[jidx];
                const geometry_msgs::Transform &t = wp.transforms[jidx];
                auto it = std::find(
                    state.multi_dof_joint_state.joint_names.begin(),
                    state.multi_dof_joint_state.joint_names.end(), joint_name);
                if (it != state.multi_dof_joint_state.joint_names.end()) {
                    size_t tvidx = std::distance(
                        state.multi_dof_joint_state.joint_names.begin(), it);
                    state.multi_dof_joint_state.transforms[tvidx] = t;
                }
            }
        }

        // write the planning variables out to file
        for (size_t vidx = 0; vidx < robotModel->jointVariableCount(); ++vidx) {
            const std::string &var_name = robotModel->getPlanningJoints()[vidx];
            const bool var_is_mdof =
                false; // TODO: multi-dof joints in robot model
            if (var_is_mdof) {
            } else {
                auto it = std::find(state.joint_state.name.begin(),
                                    state.joint_state.name.end(), var_name);
                if (it != state.joint_state.name.end()) {
                    size_t tvidx =
                        std::distance(state.joint_state.name.begin(), it);
                    double vp = state.joint_state.position[tvidx];
                    ofs << vp;
                    if (vidx != robotModel->jointVariableCount() - 1) {
                        ofs << ',';
                    }
                }
            }
        }
        ofs << '\n';
    }

    return true;
}

void convertJointVariablePathToJointTrajectory(
    const std::vector<smpl::RobotState> &path,
    trajectory_msgs::JointTrajectory &traj, smpl::RobotModel *robotModel,
    std::string planning_frame) {
    traj.header.frame_id = planning_frame;
    traj.joint_names = robotModel->getPlanningJoints();
    traj.points.clear();
    traj.points.reserve(path.size());
    for (const auto &point : path) {
        trajectory_msgs::JointTrajectoryPoint traj_pt;
        traj_pt.positions = point;
        traj.points.push_back(std::move(traj_pt));
    }
}

void PrintGrid(std::ostream &o, sbpl::OccupancyGrid &grid) {
    for (int y = grid.numCellsY() - 1; y >= 0; --y) {
        for (int x = 0; x < grid.numCellsX(); ++x) {
            if (grid.getDistance(x, y, 0) <= 0) {
                o << "1 ";
            } else {
                o << "0 ";
            }
        }
        o << '\n';
    }
}

void PrintActionSpace(const smpl::ManipLatticeActionSpace &aspace) {
    ROS_INFO("Action Set:");
    for (int i = 0; i < smpl::MotionPrimitive::Type::NUMBER_OF_MPRIM_TYPES;
         ++i) {
        smpl::MotionPrimitive::Type prim((smpl::MotionPrimitive::Type)i);
        ROS_INFO("  %s: %s @ %0.3f", to_string(prim).c_str(),
                 aspace.useAmp(prim) ? "true" : "false",
                 aspace.ampThresh(prim));
    }
    for (auto ait = aspace.begin(); ait != aspace.end(); ++ait) {
        ROS_INFO("  type: %s", to_string(ait->type).c_str());
        if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_RPY) {
            ROS_INFO("    enabled: %s",
                     aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_RPY)
                         ? "true"
                         : "false");
            ROS_INFO(
                "    thresh: %0.3f",
                aspace.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_RPY));
        } else if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_XYZ) {
            ROS_INFO("    enabled: %s",
                     aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ)
                         ? "true"
                         : "false");
            ROS_INFO(
                "    thresh: %0.3f",
                aspace.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ));
        } else if (ait->type ==
                   sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY) {
            ROS_INFO(
                "    enabled: %s",
                aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY)
                    ? "true"
                    : "false");
            ROS_INFO("    thresh: %0.3f",
                     aspace.ampThresh(
                         sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY));
        } else if (ait->type == sbpl::motion::MotionPrimitive::LONG_DISTANCE ||
                   ait->type == sbpl::motion::MotionPrimitive::SHORT_DISTANCE) {
            ROS_INFO("    action: %s", to_string(ait->action).c_str());
        }
    }
}
