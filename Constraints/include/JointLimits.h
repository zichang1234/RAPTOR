#ifndef JOINTLIMITS_H
#define JOINTLIMITS_H

#include <memory>

#include "Constraints.h"
#include "Trajectories.h"
#include "Utils.h"

namespace RAPTOR {

class JointLimits : public Constraints {
public:
    using VecX = Eigen::VectorXd;
    using MatX = Eigen::MatrixXd;

    // Constructor
    JointLimits() = default;

    // Constructor
    JointLimits(std::shared_ptr<Trajectories>& trajPtr_input, 
                const VecX& lowerLimits_input, 
                const VecX& upperLimits_input);

    // Destructor
    ~JointLimits() = default;

    // class methods:
        // compute constraints
    virtual void compute(const VecX& z, 
                         bool compute_derivatives = true,
                         bool compute_hessian = false) override;

        // compute constraints lower bounds and upper bounds
    virtual void compute_bounds() override;

        // print violation information
    virtual void print_violation_info() override;

    // class members:
    std::shared_ptr<Trajectories> trajPtr_;

    VecX lowerLimits;
    VecX upperLimits;
};

}; // namespace RAPTOR

#endif // JOINTLIMITS_H
