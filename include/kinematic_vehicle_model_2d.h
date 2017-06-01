#include <smpl/robot_model.h>
#include <vector>

namespace smpl = sbpl::motion;

/// \brief Defines a Robot Model for an (x, y) point robot
///
/// RobotModel base: basic requirements (variable types and limits)
///
/// ForwardKinematicsInterface: forward kinematics interface required by much
/// of smpl; trivial in this case to establish frame of reference
class KinematicVehicleModel2D
    : public virtual smpl::RobotModel,
      public virtual smpl::ForwardKinematicsInterface {
public:
    KinematicVehicleModel2D()
        : smpl::RobotModel(), smpl::ForwardKinematicsInterface() {
        const std::vector<std::string> joint_names = {"x", "y"};
        setPlanningJoints(joint_names);
    }

    /// \name Required Public Functions from ForwardKinematicsInterface
    ///@{
    bool computeFK(const smpl::RobotState &state, const std::string &name,
                   std::vector<double> &pose) override {
        return computePlanningLinkFK(state, pose);
    }

    bool computePlanningLinkFK(const smpl::RobotState &state,
                               std::vector<double> &pose) override {
        pose = {state[0], state[1], 0.0, 0.0, 0.0, 0.0};
        return true;
    }
    ///@}

    /// \name Required Public Functions from Robot Model
    ///@{
    double minPosLimit(int jidx) const override { return 0.0; }
    double maxPosLimit(int jidx) const override { return 0.0; }
    bool hasPosLimit(int jidx) const override { return false; }
    bool isContinuous(int jidx) const override { return false; }
    double velLimit(int jidx) const override { return 0.0; }
    double accLimit(int jidx) const override { return 0.0; }

    bool checkJointLimits(const smpl::RobotState &angles,
                          bool verbose = false) override {
        return true;
    }
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension *getExtension(size_t class_code) override {
        if (class_code == smpl::GetClassCode<RobotModel>() ||
            class_code == smpl::GetClassCode<ForwardKinematicsInterface>()) {
            return this;
        } else {
            return nullptr;
        }
    }
    ///@}
};
