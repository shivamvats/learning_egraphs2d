#include "utils.h"
#include <stdlib.h>

class StartGoalGenerator {
    public:
        StartGoalGenerator(int seed = 23534);
        void generateStartGoalOnCircle(smpl::RobotState &start_state,
                               smpl::RobotState &goal_state, Vector3d center, double radius);

    private:
};
