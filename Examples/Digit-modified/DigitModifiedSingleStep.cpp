#include "DigitModifiedSingleStepOptimizer.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <yaml-cpp/yaml.h>
#include <iomanip>

using namespace IDTO;
using namespace DigitModified;
using namespace Ipopt;

const std::string filepath = "../Examples/Digit-modified/data/";

int main(int argc, char* argv[]) {
    // set openmp number of threads
    int num_threads = 32; // this number is currently hardcoded
    omp_set_num_threads(num_threads);

    // Define robot model
    const std::string urdf_filename = "../Robots/digit-v3-modified/digit-v3-modified.urdf";
    
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    model.gravity.linear()(2) = -9.806;

    // manually define the joint axis of rotation
    // 1 for Rx, 2 for Ry, 3 for Rz
    // 4 for Px, 5 for Py, 6 for Pz
    // not sure how to extract this from a pinocchio model so define outside here.
    Eigen::VectorXi jtype(model.nq);
    jtype << 4, 5, 6, 1, 2, 3, 
             3, 3, -3, 3, 3, 3, 3,
             3, 3, -3, 3, 3, 3, 3;
    
    // ignore all motor dynamics
    model.rotorInertia.setZero();
    model.damping.setZero();
    model.friction.setZero();

    // load settings
    YAML::Node config;

    const double T = 0.4;
    TimeDiscretization time_discretization = Uniform;
    int N = 16;
    int degree = 5;
    
    GaitParameters gp;

    try {
        config = YAML::LoadFile("../Examples/Digit-modified/singlestep_optimization_settings.yaml");

        N = config["N"].as<int>();
        degree = config["degree"].as<int>();
        std::string time_discretization_str = config["time_discretization"].as<std::string>();
        time_discretization = (time_discretization_str == "Uniform") ? Uniform : Chebyshev;

        gp.swingfoot_midstep_z_des = config["swingfoot_midstep_z_des"].as<double>();
        gp.swingfoot_begin_y_des = config["swingfoot_begin_y_des"].as<double>();
        gp.swingfoot_end_y_des = config["swingfoot_end_y_des"].as<double>();
    } 
    catch (std::exception& e) {
        std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
    }

    std::cout << gp.swingfoot_begin_y_des << std::endl;
    std::cout << gp.swingfoot_end_y_des << std::endl;
    
    Eigen::VectorXd z = Utils::initializeEigenMatrixFromFile(filepath + "initial-digit-modified-Bezier.txt");

    // add disturbance to initial guess
    Eigen::VectorXd disturbance(z.size());
    if (argc > 1) {
        char* end = nullptr;
        std::srand((unsigned int)std::strtoul(argv[1], &end, 10));
        disturbance.setRandom();
        disturbance = disturbance * (0.2 * z.norm()) / disturbance.norm();
    }
    else {
        disturbance.setZero();
    }

    z = z + disturbance;
    
    SmartPtr<DigitModifiedSingleStepOptimizer> mynlp = new DigitModifiedSingleStepOptimizer();
    try {
	    mynlp->set_parameters(z,
                              T,
                              N,
                              time_discretization,
                              degree,
                              model,
                              jtype,
                              gp);
        mynlp->constr_viol_tol = 1e-4;
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
        app->Options()->SetNumericValue("point_perturbation_radius", 1e-2);
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

    // Run ipopt to solve the optimization problem
    double solve_time = 0;
    try {
        auto start = std::chrono::high_resolution_clock::now();

        // Ask Ipopt to solve the problem
        status = app->OptimizeTNLP(mynlp);

        auto end = std::chrono::high_resolution_clock::now();
        double solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
        std::cout << "Total solve time: " << solve_time << " seconds.\n";

        std::cout << "Data needed for comparison: " << mynlp->obj_value_copy << ' ' << mynlp->final_constr_violation << ' ' << solve_time << std::endl;
    }
    catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        throw std::runtime_error("Error solving optimization problem! Check previous error message!");
    }
    
    // Print the solution
    // if (mynlp->solution.size() == mynlp->numVars) {
    //     std::ofstream solution(filepath + "solution-digit-modified-Bezier-" + output_name + ".txt");
    //     solution << std::setprecision(20);
    //     for (int i = 0; i < mynlp->numVars; i++) {
    //         solution << mynlp->solution[i] << std::endl;
    //     }
    //     solution.close();

    //     std::ofstream trajectory(filepath + "trajectory-digit-modified-Bezier-" + output_name + ".txt");
    //     trajectory << std::setprecision(20);
    //     for (int i = 0; i < NUM_JOINTS; i++) {
    //         for (int j = 0; j < N; j++) {
    //             trajectory << mynlp->cidPtr_->q(j)(i) << ' ';
    //         }
    //         trajectory << std::endl;
    //     }
    //     for (int i = 0; i < NUM_JOINTS; i++) {
    //         for (int j = 0; j < N; j++) {
    //             trajectory << mynlp->cidPtr_->v(j)(i) << ' ';
    //         }
    //         trajectory << std::endl;
    //     }
    //     for (int i = 0; i < NUM_JOINTS; i++) {
    //         for (int j = 0; j < N; j++) {
    //             trajectory << mynlp->cidPtr_->a(j)(i) << ' ';
    //         }
    //         trajectory << std::endl;
    //     }
    //     for (int i = 0; i < NUM_INDEPENDENT_JOINTS; i++) {
    //         for (int j = 0; j < N; j++) {
    //             trajectory << mynlp->cidPtr_->tau(j)(i) << ' ';
    //         }
    //         trajectory << std::endl;
    //     }
    //     for (int i = 0; i < NUM_DEPENDENT_JOINTS; i++) {
    //         for (int j = 0; j < N; j++) {
    //             trajectory << mynlp->cidPtr_->lambda(j)(i) << ' ';
    //         }
    //         trajectory << std::endl;
    //     }
    //     trajectory.close();
    // }

    return 0;
}
