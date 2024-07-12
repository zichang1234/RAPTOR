
#ifndef DIGIT_CONSTRAINED_INVERSE_DYNAMICS_H
#define DIGIT_CONSTRAINED_INVERSE_DYNAMICS_H

#include "ConstrainedInverseDynamics.h"

#include "DigitDynamicsConstraints.h"
#include "DigitDynamicsConstraints.h"

namespace IDTO {
namespace Digit {

class DigitConstrainedInverseDynamics : public ConstrainedInverseDynamics {
public:
    using Model = pinocchio::Model;

    // Constructor
    DigitConstrainedInverseDynamics() = default;

    // Constructor (for single step optimization)
    DigitConstrainedInverseDynamics(const Model& model_input, 
                                    std::shared_ptr<Trajectories>& trajPtr_input,
                                    int numDependentJoints_input,
                                    const Eigen::VectorXi& jtype_input, 
                                    char stanceLeg, 
                                    const Transform& stance_foot_T_des_input);

    // Constructor (for multiple step optimization)
    DigitConstrainedInverseDynamics(const Model& model_input, 
                                    std::shared_ptr<Trajectories>& trajPtr_input,
                                    int numDependentJoints_input,
                                    const Eigen::VectorXi& jtype_input, 
                                    char stanceLeg, 
                                    const Transform& stance_foot_T_des_input,
                                    const int N_input,
                                    const int NSteps_input);

    // Destructor
    ~DigitConstrainedInverseDynamics() = default;

    // class members:
    // a pointer type of DigitDynamicsConstraints, 
    // that shares the same memory with dcPtr_ defined in base class ConstrainedInverseDynamics
    // so that other Digit-related class can access specific field in DigitDynamicsConstraints
    // such as stanceLeg, stance_foot_T_des, etc.
    std::shared_ptr<DigitDynamicsConstraints> ddcPtr_;
};

}; // namespace Digit
}; // namespace IDTO

#endif // DIGIT_CONSTRAINED_INVERSE_DYNAMICS_H
