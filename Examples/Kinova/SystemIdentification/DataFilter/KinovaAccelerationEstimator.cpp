#include "DataFilterOptimizer.h" 

using namespace IDTO;
using namespace Kinova;
using namespace Ipopt;

int main () {
    // set openmp number of threads
    int num_threads = 32; // this number is currently hardcoded
    omp_set_num_threads(num_threads);

    // Define Data
    Eigen::MatrixXd temp;
    temp = Utils::initializeEigenMatrixFromFile("../Examples/Kinova/SystemIdentification/DataFilter/data/feedback_pos.csv");
    Eigen::MatrixXd q_data = temp.transpose();
    temp = Utils::initializeEigenMatrixFromFile("../Examples/Kinova/SystemIdentification/DataFilter/data/feedback_vel.csv");
    Eigen::MatrixXd q_d_data = temp.transpose();
    temp = Utils::initializeEigenMatrixFromFile("../Examples/Kinova/SystemIdentification/DataFilter/data/frame_ts.csv");
    Eigen::VectorXd tspan = temp.array() - temp(0);

    // Define trajectory parameters
    const int degree = 10;
    const int base_frequency = 5;

    // Define initial guess
    Eigen::VectorXd z = 2 * 1.0 * Eigen::VectorXd::Random((2 * degree + 1) * 7).array() - 1.0;

    // Initialize Kinova optimizer
    SmartPtr<DataFilterOptimizer> mynlp = new DataFilterOptimizer();
    try {
	    mynlp->set_parameters(z,
                              tspan.head(1000),
                              q_data.leftCols(1000),
                              q_d_data.leftCols(1000),
                              degree,
                              base_frequency);
    }
    catch (int errorCode) {
        throw std::runtime_error("Error initializing Ipopt class! Check previous error message!");
    }

    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetNumericValue("tol", 1e-6);
    app->Options()->SetNumericValue("constr_viol_tol", 1e-6);
	app->Options()->SetNumericValue("max_wall_time", 10.0);
	app->Options()->SetIntegerValue("print_level", 5);
    // app->Options()->SetIntegerValue("max_iter", 50);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("linear_solver", "ma57");
	app->Options()->SetStringValue("hessian_approximation", "exact");

    // // For gradient checking
    // app->Options()->SetStringValue("output_file", "ipopt.out");
    // app->Options()->SetStringValue("derivative_test", "second-order");
    // app->Options()->SetNumericValue("derivative_test_perturbation", 1e-7);
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
        solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Total solve time: " << solve_time << " milliseconds.\n";
    }
    catch (int errorCode) {
        throw std::runtime_error("Error solving optimization problem! Check previous error message!");
    }

    // Print the solution
    std::ofstream filtered_position("../Examples/Kinova/SystemIdentification/DataFilter/data/filtered_pos.csv");
    std::ofstream filtered_velocity("../Examples/Kinova/SystemIdentification/DataFilter/data/filtered_vel.csv");
    std::ofstream filtered_acceleration("../Examples/Kinova/SystemIdentification/DataFilter/data/filtered_acc.csv");

    const auto trajPtr_ = mynlp->trajPtr_;
    for (int i = 0; i < trajPtr_->N; i++) {
        filtered_position << trajPtr_->q(i).transpose() << std::endl;
        filtered_velocity << trajPtr_->q_d(i).transpose() << std::endl;
        filtered_acceleration << trajPtr_->q_dd(i).transpose() << std::endl;
    }

    return 0;   
}