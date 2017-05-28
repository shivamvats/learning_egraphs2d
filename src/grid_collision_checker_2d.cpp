#include "grid_collision_checker_2d.h"

bool GridCollisionChecker2D::isStateValid(
    const smpl::RobotState& state,
    bool verbose,
    bool visualize,
    double& dist)
{
    if (state.size() < 2) {
        ROS_ERROR("State contains insufficient data");
        return false;
    }
    double x = state[0];
    double y = state[1];
    double z = 0.0;
    if (!m_grid->isInBounds(x, y, z)) {
        ROS_DEBUG("state (%0.3f, %0.3f) is out of bounds", x, y);
        return false;
    }
    if (m_grid->getDistanceFromPoint(x, y, z) <= 0.0) {
        ROS_DEBUG("state (%0.3f, %0.3f) is occupied", x, y);
        return false;
    }
    return true;
}

bool GridCollisionChecker2D::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    int& path_length,
    int& num_checks,
    double& dist)
{
    std::vector<smpl::RobotState> path;
    if (!interpolatePath(start, finish, path)) {
        return false;
    }
    return std::all_of(
        path.begin(), path.end(),
        [&](const smpl::RobotState& state)
        {
            double dist;
            return isStateValid(state, false, false, dist);
        });
}

bool GridCollisionChecker2D::interpolatePath(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    std::vector<smpl::RobotState>& path)
{
    m_grid->resolution();
    const Eigen::Vector2d vstart(start[0], start[1]);
    const Eigen::Vector2d vfinish(finish[0], finish[1]);
    int num_waypoints =
            std::ceil((vfinish - vstart).norm() / m_grid->resolution());
    num_waypoints = std::max(num_waypoints, 2);
    ROS_DEBUG("interpolate path with %d waypoints", num_waypoints);
    for (int i = 0; i < num_waypoints; ++i) {
        const double alpha = (double)i / (double)(num_waypoints - 1);
        Eigen::Vector2d vinterm = (1.0 - alpha) * vstart + alpha * vfinish;
        smpl::RobotState istate(2);
        istate[0] = vinterm.x();
        istate[1] = vinterm.y();
        path.push_back(std::move(istate));
    }
    return true;
}
