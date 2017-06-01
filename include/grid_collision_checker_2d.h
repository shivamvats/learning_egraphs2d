#ifndef GRID_COLLISION_2D_H
#define GRID_COLLISION_2D_H

#include <ros/ros.h>
#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>
#include <smpl/robot_model.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace smpl = sbpl::motion;

/// \brief Defines a collision checker for an (x,y) point robot in a grid world.
class GridCollisionChecker2D : public smpl::CollisionChecker {
public:
    GridCollisionChecker2D(sbpl::OccupancyGrid *grid) : m_grid(grid) {}

    bool isStateValid(const smpl::RobotState &state, bool verbose,
                      bool visualize, double &dist) override;

    bool isStateToStateValid(const smpl::RobotState &start,
                             const smpl::RobotState &finish, int &path_length,
                             int &num_checks, double &dist) override;

    bool interpolatePath(const smpl::RobotState &start,
                         const smpl::RobotState &finish,
                         std::vector<smpl::RobotState> &path) override;

    visualization_msgs::MarkerArray
    getCollisionModelVisualization(const smpl::RobotState &state) {
        return visualization_msgs::MarkerArray();
    }

    visualization_msgs::MarkerArray
    getVisualization(const smpl::RobotState &state) {
        return visualization_msgs::MarkerArray();
    }

private:
    // a bit heavy-weight for this, since it overlays a distance transform
    sbpl::OccupancyGrid *m_grid;
};

#endif
