#include "ConstrainedInverseDynamics.h"

namespace IDTO {

ConstrainedInverseDynamics::ConstrainedInverseDynamics(const Model& model_input, 
                                                       int N_input, 
                                                       int numDependentJoints_input,
                                                       std::unique_ptr<DynamicsConstraints>& dynamics_constraints_input) : 
    InverseDynamics(model_input, N_input),
    numDependentJoints(numDependentJoints_input) {
    numIndependentJoints = modelPtr_->nv - numDependentJoints;

    dynamicsConstraintsPtr_ = std::move(dynamics_constraints_input);

    J_dep = MatX::Zero(numDependentJoints, numDependentJoints);
    J_indep = MatX::Zero(numDependentJoints, numIndependentJoints);

    tau_dep = VecX::Zero(numDependentJoints);
    tau_indep = VecX::Zero(numIndependentJoints);

    lambda.resize(1, N);

    plambda_pq.resize(1, N);
    plambda_pv.resize(1, N);
    plambda_pa.resize(1, N);
}

void ConstrainedInverseDynamics::setupJointPositionVelocityAcceleration(VecX& q, VecX& v, VecX& a, bool compute_derivatives) {
    setupJointPosition(q);

    dynamicsConstraintsPtr_->get_c(q);
    dynamicsConstraintsPtr_->get_J(q);

    dynamicsConstraintsPtr_->get_dependent_columns(J_dep, dynamicsConstraintsPtr_->J);
    dynamicsConstraintsPtr_->get_independent_columns(J_indep, dynamicsConstraintsPtr_->J);

    J_dep_qr = QRSolver(J_dep);
    J_dep_T_qr = QRSolver(J_dep.transpose());

    // sanity check on uniqueness (these two arguments are actually equivalent)
    assert(J_dep_qr.rank() == J_dep.rows() && J_dep_qr.rank() == J_dep.cols());
    assert(J_dep_T_qr.rank() == J_dep.rows() && J_dep_T_qr.rank() == J_dep.cols());

    MatX P_dep = -J_dep_qr.solve(J_indep);

    // fill in unactuated joints velocities
    dynamicsConstraintsPtr_->fill_dependent_vector(v, P_dep * dynamicsConstraintsPtr_->get_independent_vector(v));

    dynamicsConstraintsPtr_->get_Jx_partial_dq(q, v);

    // fill in unactuated joints accelerations
    dynamicsConstraintsPtr_->fill_dependent_vector(a, -J_dep_qr.solve(J_indep * dynamicsConstraintsPtr_->get_independent_vector(a) + 
                                                   dynamicsConstraintsPtr_->Jx_partial_dq * v));

    if (compute_derivatives) {

    }      
}

void ConstrainedInverseDynamics::compute(Eigen::Array<VecX, 1, Eigen::Dynamic>& q, 
                                         Eigen::Array<VecX, 1, Eigen::Dynamic>& v, 
                                         Eigen::Array<VecX, 1, Eigen::Dynamic>& a,
                                         bool compute_derivatives) {
    for (int i = 0; i < N; i++) {                        
        setupJointPositionVelocityAcceleration(q(i), v(i), a(i), compute_derivatives);

        if (!compute_derivatives) {
            pinocchio::rnea(*modelPtr_, *dataPtr_, q(i), v(i), a(i));
        }
        else {
            pinocchio::computeRNEADerivatives(*modelPtr_, *dataPtr_, q(i), v(i), a(i), 
                                              rnea_partial_dq, rnea_partial_dv, rnea_partial_da);
        }

        // adjust with damping force and rotor inertia force
        tau(i) = dataPtr_->tau + 
                 modelPtr_->damping.cwiseProduct(v(i)) + 
                 modelPtr_->rotorInertia.cwiseProduct(a(i));
                
        if (compute_derivatives) {
            // rnea_partial_da is just the inertia matrix.
            // pinocchio only computes the upper triangle part of it.
            for (int mi = 0; mi < rnea_partial_da.rows(); mi++) {
                for (int mj = 0; mj <= mi; mj++) {
                    if (mi == mj) {
                        rnea_partial_da(mi, mj) += modelPtr_->rotorInertia(mi);
                    }
                    else {
                        rnea_partial_da(mi, mj) = rnea_partial_da(mj, mi);
                    }
                }
            }

            // pinocchio rnea does not take damping into account
            for (int mi = 0; mi < rnea_partial_da.rows(); mi++) {
                rnea_partial_dv(mi, mi) += modelPtr_->damping(mi);
            }
        }

        tau_dep = dynamicsConstraintsPtr_->get_dependent_vector(tau(i));
        tau_indep = dynamicsConstraintsPtr_->get_independent_vector(tau(i));

        // assume setupJointPositionVelocityAcceleration() has been called
        lambda(i) = J_dep_T_qr.solve(tau_dep);

        if (compute_derivatives) {

        }

        // assume setupJointPositionVelocityAcceleration() has been called
        tau(i) = tau_indep - J_indep.transpose() * lambda(i);

        if (compute_derivatives) {

        }
    }
}

}; // namespace IDTO

