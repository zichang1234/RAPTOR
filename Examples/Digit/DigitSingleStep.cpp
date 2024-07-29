#include "DigitSingleStepOptimizer.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <yaml-cpp/yaml.h>
#include <iomanip>

using namespace RAPTOR;
using namespace Digit;
using namespace Ipopt;

const std::string filepath = "../Examples/Digit/data/";

int main(int argc, char* argv[]) {
    // set openmp number of threads
    int num_threads = 32; // this number is currently hardcoded
    omp_set_num_threads(num_threads);

    // define robot model
    const std::string urdf_filename = "../Robots/digit-v3/digit-v3-armfixedspecific-floatingbase-springfixed.urdf";
    
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

    // load settings
    YAML::Node config;

    const double T = 0.4;
    TimeDiscretization time_discretization = Uniform;
    int N = 14;
    int degree = 5;
    
    GaitParameters gp;

    try {
        config = YAML::LoadFile("../Examples/Digit/singlestep_optimization_settings.yaml");

        N = config["N"].as<int>();
        degree = config["degree"].as<int>();
        std::string time_discretization_str = config["time_discretization"].as<std::string>();
        time_discretization = (time_discretization_str == "Uniform") ? Uniform : Chebyshev;

        gp.swingfoot_midstep_z_des = config["step_height"].as<double>();
        gp.swingfoot_begin_y_des = config["step_length"].as<double>() * 0.5;
        gp.swingfoot_end_y_des = -config["step_length"].as<double>() * 0.5;
    } 
    catch (std::exception& e) {
        std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
    }
    
    Eigen::VectorXd z = Utils::initializeEigenMatrixFromFile(filepath + "initial-digit-Bezier-14-5-Uniform.txt");
    // if (argc > 1) {
    //     char* end = nullptr;
    //     std::srand((unsigned int)std::strtoul(argv[1], &end, 10));
    // }
    // else {
    //     std::srand(std::time(nullptr));
    // }
    // Eigen::VectorXd z = 0.2 * Eigen::VectorXd::Random((degree + 1) * NUM_INDEPENDENT_JOINTS + NUM_JOINTS + NUM_DEPENDENT_JOINTS).array() - 0.1;
    

    SmartPtr<DigitSingleStepOptimizer> mynlp = new DigitSingleStepOptimizer();
    try {
	    mynlp->set_parameters(z,
                              T,
                              N,
                              time_discretization,
                              degree,
                              model,
                              jtype,
                              gp);
        mynlp->constr_viol_tol = config["constr_viol_tol"].as<double>();
    }
    catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        throw std::runtime_error("Error initializing Ipopt class! Check previous error message!");
    }

    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    try {
        app->Options()->SetNumericValue("tol", config["tol"].as<double>());
        app->Options()->SetNumericValue("constr_viol_tol", mynlp->constr_viol_tol);
        app->Options()->SetNumericValue("max_wall_time", config["max_wall_time"].as<double>());
        app->Options()->SetIntegerValue("max_iter", config["max_iter"].as<int>());
        app->Options()->SetNumericValue("obj_scaling_factor", config["obj_scaling_factor"].as<double>());
        app->Options()->SetIntegerValue("print_level", config["print_level"].as<double>());
        app->Options()->SetStringValue("mu_strategy", config["mu_strategy"].as<std::string>().c_str());
        app->Options()->SetStringValue("linear_solver", config["linear_solver"].as<std::string>().c_str());
        app->Options()->SetStringValue("ma57_automatic_scaling", "yes");

        if (mynlp->enable_hessian) {
            app->Options()->SetStringValue("hessian_approximation", "exact");
        }
        else {
            app->Options()->SetStringValue("hessian_approximation", "limited-memory");
        }
        // app->Options()->SetStringValue("nlp_scaling_method", "none");
    }
    catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        throw std::runtime_error("Error setting optimization options! Check previous error message!");
    }

    if (config["gredient_check"].as<bool>()) {
        app->Options()->SetStringValue("output_file", "ipopt.out");
        if (mynlp->enable_hessian) {
            app->Options()->SetStringValue("derivative_test", "second-order");
        }
        else {
            app->Options()->SetStringValue("derivative_test", "first-order");
        }
        app->Options()->SetNumericValue("point_perturbation_radius", 1e-3);
        // app->Options()->SetIntegerValue("derivative_test_first_index", 168);
        app->Options()->SetNumericValue("derivative_test_perturbation", 1e-7);
        app->Options()->SetNumericValue("derivative_test_tol", 1e-4);
    }

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
        double solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;

        std::cout << "Data needed for comparison: " << mynlp->obj_value_copy << ' ' << mynlp->final_constr_violation << ' ' << solve_time << std::endl;
    }
    catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        throw std::runtime_error("Error solving optimization problem! Check previous error message!");
    }

    // Print the solution
    const int N_reachsets = 128;
    if (mynlp->solution.size() == mynlp->numVars) {
        // Evaluate the solution on a finer time discretization
        try {
            SmartPtr<DigitSingleStepOptimizer> testnlp = new DigitSingleStepOptimizer();
            testnlp->set_parameters(z,
                                    T,
                                    N_reachsets,
                                    TimeDiscretization::Uniform,
                                    degree,
                                    model,
                                    jtype,
                                    gp);
            Index n, m, nnz_jac_g, nnz_h_lag;
            TNLP::IndexStyleEnum index_style;
            testnlp->get_nlp_info(n, m, nnz_jac_g, nnz_h_lag, index_style);
            Number ztry[testnlp->numVars], x_l[testnlp->numVars], x_u[testnlp->numVars];
            Number g[testnlp->numCons], g_lb[testnlp->numCons], g_ub[testnlp->numCons];
            for (int i = 0; i < testnlp->numVars; i++) {
                ztry[i] = mynlp->solution[i];
            }
            testnlp->get_bounds_info(testnlp->numVars, x_l, x_u, testnlp->numCons, g_lb, g_ub);
            testnlp->eval_g(testnlp->numVars, ztry, false, testnlp->numCons, g);

            std::ofstream trajectory(filepath + "trajectory-digit.txt");
            const auto& cidPtr_ = testnlp->cidPtr_;
            for (int i = 0; i < NUM_JOINTS; i++) {
                for (int j = 0; j < N_reachsets; j++) {
                    trajectory << cidPtr_->q(j)(i) << ' ';
                }
                trajectory << std::endl;
            }
            trajectory.close();
        }
        catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
            throw std::runtime_error("Error evaluating the solution on a finer time discretization! Check previous error message!");
        }
    }

    return 0;
}
