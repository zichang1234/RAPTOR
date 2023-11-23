#include "InverseDynamics.h"

namespace IDTO {

InverseDynamics::InverseDynamics(const Model& model_input, int N_input) :
    N(N_input) {
    modelPtr_ = std::make_unique<Model>(model_input);
    dataPtr_ = std::make_unique<Data>(model_input);

    tau.resize(1, N);

    ptau_pq.resize(1, N);
    ptau_pq_d.resize(1, N);
    ptau_pq_dd.resize(1, N);

    rnea_partial_dq = MatX::Zero(modelPtr_->nv, modelPtr_->nv);
    rnea_partial_dv = MatX::Zero(modelPtr_->nv, modelPtr_->nv);
    rnea_partial_da = MatX::Zero(modelPtr_->nv, modelPtr_->nv);
}

void InverseDynamics::compute(const Eigen::Array<VecX, 1, Eigen::Dynamic>& q, 
                              const Eigen::Array<VecX, 1, Eigen::Dynamic>& q_d, 
                              const Eigen::Array<VecX, 1, Eigen::Dynamic>& q_dd,
                              bool compute_derivatives) {
    for (int i = 0; i < N; i++) {
        if (!compute_derivatives) {
            pinocchio::rnea(*modelPtr_, *dataPtr_, q(i), q_d(i), q_dd(i));
        }
        else {
            pinocchio::computeRNEADerivatives(*modelPtr_, *dataPtr_, q(i), q_d(i), q_dd(i), 
                                              rnea_partial_dq, rnea_partial_dv, rnea_partial_da);
        }
        
        // adjust with damping force and rotor inertia force
        tau(i) = dataPtr_->tau + 
                 modelPtr_->damping.cwiseProduct(q_d(i)) + 
                 modelPtr_->rotorInertia.cwiseProduct(q_dd(i));
                
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

            ptau_pq(i) = rnea_partial_dq;
            ptau_pq_d(i) = rnea_partial_dv;
            ptau_pq_dd(i) = rnea_partial_da;
        }
    }
}

}; // namespace IDTO

