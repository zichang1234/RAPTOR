#include "DigitModifiedSingleStepOptimizer.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <iomanip>

using namespace IDTO;
using namespace DigitModified;
using namespace Ipopt;

using std::cout;
using std::endl;

int main(int argc, char* argv[]) {
    // Eigen::initParallel();

    // Define robot model
    const std::string urdf_filename = "../Examples/Digit-modified/digit-v3-modified.urdf";
    
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

    pinocchio::Data data(model);

    const double T = 0.4;
    const int N = 16;
    const int degree = 5;

    GaitParameters gp;
    gp.swingfoot_midstep_z_des = 0.22;
    gp.swingfoot_begin_x_des = -0.25;
    gp.swingfoot_begin_y_des = 0.40;
    gp.swingfoot_end_x_des = -0.25;
    gp.swingfoot_end_y_des = -0.40;                                  

    std::ifstream initial_guess("initial-digit-modified-Bezier.txt");

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

    // add disturbance to initial guess
    Eigen::VectorXd disturbance(z_array.size());
    if (argc > 1) {
        char* end = nullptr;
        std::srand((unsigned int)std::strtoul(argv[1], &end, 10));
    }
    else {
        std::srand(0);
    }
    disturbance.setRandom();
    disturbance = disturbance * (0.5 * z.norm()) / disturbance.norm();

    z = z + disturbance;
    
    SmartPtr<DigitModifiedSingleStepOptimizer> mynlp = new DigitModifiedSingleStepOptimizer();
    try {
	    mynlp->set_parameters(z,
                              T,
                              N,
                              degree,
                              model,
                              jtype,
                              gp);
    }
    catch (int errorCode) {
        throw std::runtime_error("Error initializing Ipopt class! Check previous error message!");
    }

    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetNumericValue("tol", 1);
	app->Options()->SetNumericValue("max_wall_time", 500);
    app->Options()->SetNumericValue("obj_scaling_factor", 1e-4);
    // app->Options()->SetNumericValue("constr_viol_tol", 1e-4);
    // app->Options()->SetNumericValue("dual_inf_tol", 10);
    // app->Options()->SetNumericValue("acceptable_constr_viol_tol", 1e-4);
    // app->Options()->SetNumericValue("acceptable_dual_inf_tol", 10);
    app->Options()->SetIntegerValue("max_iter", 2000);
	app->Options()->SetIntegerValue("print_level", 5);
    app->Options()->SetStringValue("mu_strategy", "monotone");
    app->Options()->SetStringValue("linear_solver", "ma57");
	app->Options()->SetStringValue("hessian_approximation", "limited-memory");

    app->Options()->SetStringValue("nlp_scaling_method", "none");

    // For gradient checking
    // const std::string outputfilename = "../data/Digit-modified/ipopt_digit-modified-Bezier-" + std::string(argv[1]) + ".out";
    // app->Options()->SetStringValue("output_file", outputfilename.c_str());
    // app->Options()->SetStringValue("derivative_test", "first-order");
    // app->Options()->SetNumericValue("point_perturbation_radius", 1e-2);
    // // app->Options()->SetIntegerValue("derivative_test_first_index", 168);
    // app->Options()->SetNumericValue("derivative_test_perturbation", 1e-6);
    // app->Options()->SetNumericValue("derivative_test_tol", 1e-4);

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
        // solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
        solve_time = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
        std::cout << "Total solve time: " << solve_time << " seconds.\n";
    }
    catch (int errorCode) {
        throw std::runtime_error("Error solving optimization problem! Check previous error message!");
    }
    
    // Print the solution
    if (mynlp->solution.size() == mynlp->numVars) {
        std::ofstream solution("solution-digit-modified-Bezier.txt");
        solution << std::setprecision(20);
        for (int i = 0; i < mynlp->numVars; i++) {
            solution << mynlp->solution[i] << std::endl;
        }
        solution.close();

        std::ofstream trajectory("trajectory-digit-modified-Bezier.txt");
        trajectory << std::setprecision(20);
        for (int i = 0; i < NUM_JOINTS; i++) {
            for (int j = 0; j < N; j++) {
                trajectory << mynlp->cidPtr_->q(j)(i) << ' ';
            }
            trajectory << std::endl;
        }
        for (int i = 0; i < NUM_JOINTS; i++) {
            for (int j = 0; j < N; j++) {
                trajectory << mynlp->cidPtr_->v(j)(i) << ' ';
            }
            trajectory << std::endl;
        }
        for (int i = 0; i < NUM_JOINTS; i++) {
            for (int j = 0; j < N; j++) {
                trajectory << mynlp->cidPtr_->a(j)(i) << ' ';
            }
            trajectory << std::endl;
        }
        for (int i = 0; i < NUM_INDEPENDENT_JOINTS; i++) {
            for (int j = 0; j < N; j++) {
                trajectory << mynlp->cidPtr_->tau(j)(i) << ' ';
            }
            trajectory << std::endl;
        }
        for (int i = 0; i < NUM_DEPENDENT_JOINTS; i++) {
            for (int j = 0; j < N; j++) {
                trajectory << mynlp->cidPtr_->lambda(j)(i) << ' ';
            }
            trajectory << std::endl;
        }
        trajectory.close();
    }

    // Number x[z.size()];
    // for (int i = 0; i < z.size(); i++) {
    //     x[i] = z(i);
    // }
    // Index n, m, nnz_jac_g, nnz_h_lag;
    // TNLP::IndexStyleEnum index_style;
    // mynlp->get_nlp_info(n, m, nnz_jac_g, nnz_h_lag, index_style);
    // Number f;
    // Number grad_f[mynlp->numVars];
    // mynlp->eval_f(mynlp->numVars, x, false, f);
    // mynlp->eval_grad_f(mynlp->numVars, x, false, grad_f);
    // Number g[mynlp->numCons];
    // Number values[mynlp->numCons * mynlp->numVars];
    // mynlp->eval_g(mynlp->numVars, x, false, mynlp->numCons, g);
    // mynlp->eval_jac_g(mynlp->numVars, x, false, mynlp->numCons, 0, NULL, NULL, values);

    return 0;
}