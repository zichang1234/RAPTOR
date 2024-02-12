#include "KinovaOptimizer.h"

namespace IDTO {
namespace Kinova {

// // constructor
// KinovaOptimizer::KinovaOptimizer()
// {
// }


// // destructor
// KinovaOptimizer::~KinovaOptimizer()
// {
// }


bool KinovaOptimizer::set_parameters(
    const VecX& x0_input,
    const double T_input,
    const int N_input,
    const int degree_input,
    const Model& model_input, 
    const Eigen::VectorXi& jtype_input,
    const ArmourTrajectoryParameters& atp_input,
    const Eigen::Array<Vec3, 1, Eigen::Dynamic>& zonotopeCenters_input,
    const Eigen::Array<MatX, 1, Eigen::Dynamic>& zonotopeGenerators_input,
    const VecX& qdes_input,
    const int tplan_n_input,
    const VecX& joint_limits_buffer_input,
    const VecX& velocity_limits_buffer_input,
    const VecX& torque_limits_buffer_input
 ) 
{
    x0 = x0_input;
    qdes = qdes_input;
    tplan_n = tplan_n_input;

    trajPtr_ = std::make_shared<ArmourBezierCurves>(T_input, 
                                                    N_input, 
                                                    model_input.nq, 
                                                    Chebyshev, 
                                                    atp_input);

    idPtr_ = std::make_shared<InverseDynamics>(model_input,
                                               trajPtr_);
    
    // read joint limits from KinovaConstants.h
    VecX JOINT_LIMITS_LOWER_VEC(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++) {
        JOINT_LIMITS_LOWER_VEC(i) = JOINT_LIMITS_LOWER[i];
    }
    JOINT_LIMITS_LOWER_VEC = JOINT_LIMITS_LOWER_VEC + joint_limits_buffer_input;

    VecX JOINT_LIMITS_UPPER_VEC(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++) {
        JOINT_LIMITS_UPPER_VEC(i) = JOINT_LIMITS_UPPER[i];
    }
    JOINT_LIMITS_UPPER_VEC = JOINT_LIMITS_UPPER_VEC - joint_limits_buffer_input;

    // read velocity limits from KinovaConstants.h
    VecX VELOCITY_LIMITS_LOWER_VEC(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++) {
        VELOCITY_LIMITS_LOWER_VEC(i) = VELOCITY_LIMITS_LOWER[i];
    }
    VELOCITY_LIMITS_LOWER_VEC = VELOCITY_LIMITS_LOWER_VEC + velocity_limits_buffer_input;

    VecX VELOCITY_LIMITS_UPPER_VEC(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++) {
        VELOCITY_LIMITS_UPPER_VEC(i) = VELOCITY_LIMITS_UPPER[i];
    }
    VELOCITY_LIMITS_UPPER_VEC = VELOCITY_LIMITS_UPPER_VEC - velocity_limits_buffer_input;

    // read torque limits from KinovaConstants.h
    VecX TORQUE_LIMITS_LOWER_VEC(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++) {
        TORQUE_LIMITS_LOWER_VEC(i) = TORQUE_LIMITS_LOWER[i];
    }
    TORQUE_LIMITS_LOWER_VEC = TORQUE_LIMITS_LOWER_VEC + torque_limits_buffer_input;

    VecX TORQUE_LIMITS_UPPER_VEC(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++) {
        TORQUE_LIMITS_UPPER_VEC(i) = TORQUE_LIMITS_UPPER[i];
    }
    TORQUE_LIMITS_UPPER_VEC = TORQUE_LIMITS_UPPER_VEC - torque_limits_buffer_input;

    // Joint limits
    constraintsPtrVec_.push_back(std::make_unique<JointLimits>(trajPtr_, 
                                                               JOINT_LIMITS_LOWER_VEC, 
                                                               JOINT_LIMITS_UPPER_VEC));
    constraintsNameVec_.push_back("joint limits");

    // Velocity limits
    constraintsPtrVec_.push_back(std::make_unique<VelocityLimits>(trajPtr_, 
                                                                  VELOCITY_LIMITS_LOWER_VEC, 
                                                                  VELOCITY_LIMITS_UPPER_VEC));
    constraintsNameVec_.push_back("velocity limits");        

    // Torque limits
    constraintsPtrVec_.push_back(std::make_unique<TorqueLimits>(trajPtr_, 
                                                                idPtr_,
                                                                TORQUE_LIMITS_LOWER_VEC, 
                                                                TORQUE_LIMITS_UPPER_VEC));
    constraintsNameVec_.push_back("torque limits");                                                            

    // Customized constraints (collision avoidance with obstacles)
    constraintsPtrVec_.push_back(std::make_unique<KinovaCustomizedConstraints>(trajPtr_,
                                                                               model_input,
                                                                               jtype_input,
                                                                               zonotopeCenters_input,
                                                                               zonotopeGenerators_input));   
    constraintsNameVec_.push_back("obstacle avoidance constraints");                                                                                                                                                                                            
                                                                                                                                                                                                                                                                                                                                                                        
    assert(x0.size() == trajPtr_->varLength);
    assert(qdes.size() == trajPtr_->Nact);
    assert(tplan_n >= 0 && tplan_n < trajPtr_->N);

    return true;
}
// [TNLP_set_parameters]

bool KinovaOptimizer::get_nlp_info(
   Index&          n,
   Index&          m,
   Index&          nnz_jac_g,
   Index&          nnz_h_lag,
   IndexStyleEnum& index_style
)
{
    // number of decision variables
    numVars = trajPtr_->varLength;
    n = numVars;

    // number of inequality constraint
    numCons = 0;
    for ( Index i = 0; i < constraintsPtrVec_.size(); i++ ) {
        numCons += constraintsPtrVec_[i]->m;
    }
    m = numCons;

    nnz_jac_g = n * m;
    nnz_h_lag = n * n;

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

    return true;
}
// [TNLP_get_nlp_info]

// [TNLP_eval_f]
// returns the value of the objective function
bool KinovaOptimizer::eval_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number&       obj_value
)
{
    if(n != numVars){
       throw std::runtime_error("*** Error wrong value of n in eval_f!");
    }

    VecX z(n);
    for ( Index i = 0; i < n; i++ ) {
        z(i) = x[i];
    }

    trajPtr_->compute(z, false);

    const VecX& qplan = trajPtr_->q(tplan_n);

    obj_value = pow(wrapToPi(qplan[0] - qdes[0]), 2) + // These are continuous joints
                pow(wrapToPi(qplan[2] - qdes[2]), 2) + 
                pow(wrapToPi(qplan[4] - qdes[4]), 2) + 
                pow(wrapToPi(qplan[6] - qdes[6]), 2) + 
                pow(qplan[1] - qdes[1], 2) +           // These are not continuous joints
                pow(qplan[3] - qdes[3], 2) + 
                pow(qplan[5] - qdes[5], 2);

    return true;
}
// [TNLP_eval_f]

// [TNLP_eval_grad_f]
// return the gradient of the objective function grad_{x} f(x)
bool KinovaOptimizer::eval_grad_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number*       grad_f
)
{
    if(n != numVars){
       throw std::runtime_error("*** Error wrong value of n in eval_f!");
    }

    VecX z(n);
    for ( Index i = 0; i < n; i++ ) {
        z(i) = x[i];
    }

    trajPtr_->compute(z, true);

    const VecX& qplan = trajPtr_->q(tplan_n);
    const MatX& pqplan_pz = trajPtr_->pq_pz(tplan_n);

    for(Index i = 0; i < n; i++){
        if (i % 2 == 0) {
            grad_f[i] = (2 * wrapToPi(qplan[i] - qdes[i]) * pqplan_pz(i, i));
        }
        else {
            grad_f[i] = (2 * (qplan[i] - qdes[i]) * pqplan_pz(i, i));
        }
    }

    return true;
}
// [TNLP_eval_grad_f]

}; // namespace Kinova
}; // namespace IDTO