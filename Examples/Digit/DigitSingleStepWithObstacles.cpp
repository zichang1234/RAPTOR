#include "DigitSingleStepOptimizerWithObstacles.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <iomanip>

using namespace IDTO;
using namespace Digit;
using namespace Ipopt;

using std::cout;
using std::endl;

int main(int argc, char* argv[]) {
    // Eigen::initParallel();

    // define robot model
    const std::string urdf_filename = "../Examples/Digit/digit-v3-armfixedspecific-floatingbase-springfixed.urdf";
    
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    model.gravity.linear()(2) = -9.806;

    // manually define the joint axis of rotation
    // 1 for Rx, 2 for Ry, 3 for Rz
    // 4 for Px, 5 for Py, 6 for Pz
    // not sure how to extract this from a pinocchio model so define outside here.
    Eigen::VectorXi jtype(model.nq);
    jtype << 4, 5, 6, 1, 2, 3, 
             3, 3, -3, 3, 2, 3, 3, 3, 3, 2, 3, 3, 2, 3, 3,
             3, 3, -3, 3, 2, 3, 3, 3, 3, 2, 3, 3, 2, 3, 3;
    
    // ignore friction for now
    model.friction.setZero();

    // manually import motor inertia 
    model.rotorInertia(model.getJointId("left_hip_roll") - 1) = 0.173823936;
    model.rotorInertia(model.getJointId("left_hip_yaw") - 1) = 0.067899975;
    model.rotorInertia(model.getJointId("left_hip_pitch") - 1) = 0.1204731904;
    model.rotorInertia(model.getJointId("left_knee") - 1) = 0.1204731904;
    model.rotorInertia(model.getJointId("left_toe_A") - 1) = 0.036089475;
    model.rotorInertia(model.getJointId("left_toe_B") - 1) = 0.036089475;
    model.rotorInertia(model.getJointId("right_hip_roll") - 1) = 0.173823936;
    model.rotorInertia(model.getJointId("right_hip_yaw") - 1) = 0.067899975;
    model.rotorInertia(model.getJointId("right_hip_pitch") - 1) = 0.1204731904;
    model.rotorInertia(model.getJointId("right_knee") - 1) = 0.1204731904;
    model.rotorInertia(model.getJointId("right_toe_A") - 1) = 0.036089475;
    model.rotorInertia(model.getJointId("right_toe_B") - 1) = 0.036089475;

    pinocchio::Data data(model);

    const double T = 0.4;
    const TimeDiscretization time_discretization = Uniform;
    const int N = 14;
    const int degree = 5;

    const int num_obstacles = 1;
    Eigen::Array<Eigen::Vector3d, 1, num_obstacles> zonotopeCenters;
    Eigen::Array<Eigen::MatrixXd, 1, num_obstacles> zonotopeGenerators;

    for (int i = 0; i < num_obstacles; i++) {
        zonotopeCenters(i) << 0, 0, 0;
        zonotopeGenerators(i) = 0.1 * Eigen::MatrixXd::Identity(3, 3);
    }

    Eigen::VectorXd q_act0(NUM_INDEPENDENT_JOINTS);
    Eigen::VectorXd q_act_d0(NUM_INDEPENDENT_JOINTS);
    q_act0 << -0.055006, -0.14424, 0.48841, 0.84331, -0.53871, 0.19699, 0.067928, 0.41611, -0.49578, -0.85886, 0.39913, -0.0887;
    q_act_d0 << 0.0063512, 0.033545, -0.021454, -0.0065737, 0.15027, 0.24112, -0.053465, -0.0014915, -0.010528, 0.0045807, 0.14383, 0.20103;

    std::ifstream initial_guess("initial-digit-Bezier.txt");
    double temp = 0;
    std::vector<double> z_array;
    while (initial_guess >> temp) {
        z_array.push_back(temp);
    }
    initial_guess.close();
    Eigen::VectorXd z(z_array.size());
    for (int i = 0; i < z_array.size(); i++) {
        z(i) = z_array[i];
    }
  
    SmartPtr<DigitSingleStepOptimizerWithObstacles> mynlp = new DigitSingleStepOptimizerWithObstacles();
    try {
	    mynlp->set_parameters(z,
                              T,
                              N,
                              time_discretization,
                              degree,
                              model,
                              jtype,
                              zonotopeCenters,
                              zonotopeGenerators,
                              q_act0,
                              q_act_d0);
    }
    catch (int errorCode) {
        throw std::runtime_error("Error initializing Ipopt class! Check previous error message!");
    }

    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetNumericValue("tol", 1e-3);
	app->Options()->SetNumericValue("max_wall_time", 500);
    app->Options()->SetNumericValue("obj_scaling_factor", 1e-4);
    app->Options()->SetNumericValue("constr_viol_tol", 1e-4);
    app->Options()->SetIntegerValue("max_iter", 500);
	app->Options()->SetIntegerValue("print_level", 5);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("linear_solver", "ma57");
	app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetStringValue("nlp_scaling_method", "none");

    // For gradient checking
    app->Options()->SetStringValue("output_file", "ipopt_digit.out");
    // app->Options()->SetStringValue("derivative_test", "first-order");
    // app->Options()->SetNumericValue("point_perturbation_radius", 1e-3);
    // // app->Options()->SetIntegerValue("derivative_test_first_index", 168);
    // app->Options()->SetNumericValue("derivative_test_perturbation", 1e-7);
    // app->Options()->SetNumericValue("derivative_test_tol", 1e-4);

    // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if( status != Solve_Succeeded ) {
		throw std::runtime_error("Error during initialization of optimization!");
    }

    try {
        auto start = std::chrono::high_resolution_clock::now();

        // Ask Ipopt to solve the problem
        status = app->OptimizeTNLP(mynlp);

        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "Total solve time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0 << " seconds.\n";
    }
    catch (int errorCode) {
        throw std::runtime_error("Error solving optimization problem! Check previous error message!");
    }

    // Print the solution
    if (mynlp->solution.size() == mynlp->numVars) {
        std::ofstream solution("solution-digit-with-obstacles.txt");
        solution << std::setprecision(20);
        for (int i = 0; i < mynlp->numVars; i++) {
            solution << mynlp->solution[i] << std::endl;
        }
        solution.close();

        // std::ofstream trajectory("../data/Digit/trajectory-digit-with-obstacles.txt");
        // trajectory << std::setprecision(20);
        // for (int i = 0; i < NUM_JOINTS; i++) {
        //     for (int j = 0; j < N; j++) {
        //         trajectory << mynlp->cidPtr_->q(j)(i) << ' ';
        //     }
        //     trajectory << std::endl;
        // }
        // for (int i = 0; i < NUM_JOINTS; i++) {
        //     for (int j = 0; j < N; j++) {
        //         trajectory << mynlp->cidPtr_->v(j)(i) << ' ';
        //     }
        //     trajectory << std::endl;
        // }
        // for (int i = 0; i < NUM_JOINTS; i++) {
        //     for (int j = 0; j < N; j++) {
        //         trajectory << mynlp->cidPtr_->a(j)(i) << ' ';
        //     }
        //     trajectory << std::endl;
        // }
        // for (int i = 0; i < NUM_INDEPENDENT_JOINTS; i++) {
        //     for (int j = 0; j < N; j++) {
        //         trajectory << mynlp->cidPtr_->tau(j)(i) << ' ';
        //     }
        //     trajectory << std::endl;
        // }
        // for (int i = 0; i < NUM_DEPENDENT_JOINTS; i++) {
        //     for (int j = 0; j < N; j++) {
        //         trajectory << mynlp->cidPtr_->lambda(j)(i) << ' ';
        //     }
        //     trajectory << std::endl;
        // }
        // trajectory.close();
    }

    // try {
	//     Number ztry[mynlp->numVars];
    //     Number g[mynlp->numCons];
    //     Number obj_value = 0;
    //     for (int i = 0; i < mynlp->numVars; i++) {
    //         ztry[i] = z[i];
    //     }
    //     mynlp->eval_f(mynlp->numVars, ztry, false, obj_value);
    //     mynlp->eval_g(mynlp->numVars, ztry, false, mynlp->numCons, g);
    //     std::cout << "Objective function value: " << obj_value << std::endl;
    //     mynlp->summarize_constraints(mynlp->numCons, g);
    // }
    // catch (int errorCode) {
    //     throw std::runtime_error("Error initializing Ipopt class! Check previous error message!");
    // }
    

    return 0;
}