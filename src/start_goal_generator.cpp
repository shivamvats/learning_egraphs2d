#include "start_goal_generator.h"

StartGoalGenerator::StartGoalGenerator(int seed) {
    srand(seed);
}

void StartGoalGenerator::generateStartGoalOnCircle(smpl::RobotState &start_state,
                                              smpl::RobotState &goal_state,
                                              Vector3d center, double radius) {
    Vector3d start, goal;
    start = samplePointsOnCircle(center, radius, 1)[0];
    goal = samplePointsOnCircle(center, radius, 1)[0];

    start_state = {start[0], start[1], start[2]};
    goal_state = {goal[0], goal[1], goal[2]};
}
