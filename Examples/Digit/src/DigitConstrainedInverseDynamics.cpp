#include "DigitConstrainedInverseDynamics.h"

namespace IDTO {
namespace Digit {

DigitConstrainedInverseDynamics::DigitConstrainedInverseDynamics(const Model& model_input, 
                                                                 std::shared_ptr<Trajectories>& trajPtr_input,
                                                                 int NUM_DEPENDENT_JOINTS_input,
                                                                 const Eigen::VectorXi& jtype_input, 
                                                                 char stanceLeg_input, 
                                                                 const Transform& stance_foot_T_des_input) :
    ConstrainedInverseDynamics(model_input, trajPtr_input, NUM_DEPENDENT_JOINTS_input) {
    dcPtr_ = std::make_shared<DigitDynamicsConstraints>(modelPtr_, 
                                                        jtype_input, 
                                                        stanceLeg_input,
                                                        stance_foot_T_des_input);
}

DigitConstrainedInverseDynamics::DigitConstrainedInverseDynamics(const Model& model_input, 
                                                                 std::shared_ptr<Trajectories>& trajPtr_input,
                                                                 int NUM_DEPENDENT_JOINTS_input,
                                                                 const Eigen::VectorXi& jtype_input, 
                                                                 char stanceLeg_input, 
                                                                 const Transform& stance_foot_T_des_input,
                                                                 const int N_input,
                                                                 const int num_steps_input) :
    ConstrainedInverseDynamics(model_input, trajPtr_input, NUM_DEPENDENT_JOINTS_input) {
    dcPtr_ = std::make_shared<DigitMultipleStepDynamicsConstraints>(modelPtr_, 
                                                                    jtype_input, 
                                                                    stanceLeg_input,
                                                                    stance_foot_T_des_input,
                                                                    N_input,
                                                                    num_steps_input);
}

}; // namespace Digit
}; // namespace IDTO