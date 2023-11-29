#include "KinovaOptimizer.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

using namespace IDTO;
using namespace Kinova;
using namespace Ipopt;

using std::cout;
using std::endl;

int main() {
    // define robot model
    const std::string urdf_filename = "../Examples/Kinova/kinova.urdf";
    
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    model.gravity.linear()(2) = -9.806;

    // manually define the joint axis of rotation
    // 1 for Rx, 2 for Ry, 3 for Rz
    // 4 for Px, 5 for Py, 6 for Pz
    // not sure how to extract this from a pinocchio model so define outside here.
    Eigen::VectorXi jtype(model.nq);
    jtype << 3, 3, 3, 3, 3, 3, 3;

    const double T = 2;
    const int N = 16;
    const int degree = 6;

    Eigen::VectorXd z((2 * degree + 4) * model.nq);
    z.setConstant(15);

    SmartPtr<KinovaOptimizer> mynlp = new KinovaOptimizer();
    try {
	    mynlp->set_parameters(z,
                              T,
                              N,
                              degree,
                              model,
                              jtype);
    }
    catch (int errorCode) {
        throw std::runtime_error("Error initializing Ipopt class! Check previous error message!");
    }

    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetNumericValue("tol", 1e-6);
	app->Options()->SetNumericValue("max_wall_time", 1e-3);
	app->Options()->SetIntegerValue("print_level", 5);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("linear_solver", "ma97");
	app->Options()->SetStringValue("hessian_approximation", "limited-memory");

    // For gradient checking
    // app->Options()->SetStringValue("output_file", "ipopt.out");
    // app->Options()->SetStringValue("derivative_test", "first-order");
    // app->Options()->SetNumericValue("point_perturbation_radius", 0);
    // // app->Options()->SetIntegerValue("derivative_test_first_index", 168);
    // app->Options()->SetNumericValue("derivative_test_perturbation", 1e-7);
    // app->Options()->SetNumericValue("derivative_test_tol", 1e-5);

    // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if( status != Solve_Succeeded ) {
		throw std::runtime_error("Error during initialization of optimization!");
    }

    try {
        // Ask Ipopt to solve the problem
        status = app->OptimizeTNLP(mynlp);
    }
    catch (int errorCode) {
        throw std::runtime_error("Error solving optimization problem! Check previous error message!");
    }

    return 0;
}