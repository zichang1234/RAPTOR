#include "KinovaIKMotionPybindWrapper.h"

namespace RAPTOR {
namespace Kinova {

KinovaIKMotionPybindWrapper::KinovaIKMotionPybindWrapper(const std::string urdf_filename,
                                                         const bool display_info) {
    // Define robot model
    pinocchio::urdf::buildModel(urdf_filename, model);

    mynlp = new KinovaIKSolver();
    mynlp->display_info = display_info;

    app = IpoptApplicationFactory();
    app->Options()->SetStringValue("hessian_approximation", "exact");

    endT = Transform(Vec3(M_PI, 0, 0), Vec3(0, 0, -0.061525));
}

void KinovaIKMotionPybindWrapper::set_desired_endeffector_transforms(const nb_2d_double desired_endeffector_transforms_inp) {
    if (desired_endeffector_transforms_inp.shape(1) != 12) {
        throw std::invalid_argument("Input should be of shape (N, 12)!");
    }

    desiredTransforms.clear();
    for (size_t i = 0; i < desired_endeffector_transforms_inp.shape(0); i++) {
        const size_t shape_ptr[] = {3, 4};
        Transform desiredTransform;
        desiredTransform.p << desired_endeffector_transforms_inp(i, 0),
                              desired_endeffector_transforms_inp(i, 1),
                              desired_endeffector_transforms_inp(i, 2);
        desiredTransform.R << desired_endeffector_transforms_inp(i, 3), desired_endeffector_transforms_inp(i, 4), desired_endeffector_transforms_inp(i, 5),
                              desired_endeffector_transforms_inp(i, 6), desired_endeffector_transforms_inp(i, 7), desired_endeffector_transforms_inp(i, 8),
                              desired_endeffector_transforms_inp(i, 9), desired_endeffector_transforms_inp(i, 10), desired_endeffector_transforms_inp(i, 11);
        
        // test if it is a valid rotation matrix
        if (fabs(desiredTransform.R.determinant() - 1) > 1e-8) {
            std::cerr << desiredTransform.R << std::endl;
            throw std::invalid_argument("Input rotation matrix is not valid (Determinant not equal to 1)!");
        }
        const MatX testR = desiredTransform.R * desiredTransform.R.transpose();
        if (!testR.isApprox(Mat3::Identity(), 1e-8)) {
            throw std::invalid_argument("Input rotation matrix is not valid (Not orthogonal)!");
        }

        desiredTransforms.push_back(desiredTransform);
    }

    set_transform_check = true;
    has_optimized = false;
}

void KinovaIKMotionPybindWrapper::set_ipopt_parameters(const double tol,
                                                       const double constr_viol_tol,
                                                       const double obj_scaling_factor,
                                                       const double max_wall_time, 
                                                       const int print_level,
                                                       const std::string mu_strategy,
                                                       const std::string linear_solver,
                                                       const bool gradient_check) {
    app->Options()->SetNumericValue("tol", tol);
    app->Options()->SetNumericValue("constr_viol_tol", constr_viol_tol);
    mynlp->constr_viol_tol = constr_viol_tol;
    app->Options()->SetNumericValue("obj_scaling_factor", obj_scaling_factor);
    app->Options()->SetNumericValue("max_wall_time", max_wall_time);
    app->Options()->SetIntegerValue("print_level", print_level);
    app->Options()->SetStringValue("mu_strategy", mu_strategy);
    app->Options()->SetStringValue("linear_solver", linear_solver);
    app->Options()->SetStringValue("ma57_automatic_scaling", "yes");

    if (gradient_check) {
        app->Options()->SetStringValue("output_file", "ipopt.out");
        app->Options()->SetStringValue("derivative_test", "second-order");
        app->Options()->SetNumericValue("derivative_test_perturbation", 1e-8);
        app->Options()->SetNumericValue("derivative_test_tol", 1e-4);
    }

    set_ipopt_parameters_check = true;
    has_optimized = false;
}

nb::tuple KinovaIKMotionPybindWrapper::solve() {
    if (!set_transform_check || 
        !set_ipopt_parameters_check) {
        throw std::runtime_error("parameters not set properly!");
    }

    solutions.resize(model.nv, desiredTransforms.size());

    // Define initial guess
    VecX z = VecX::Zero(model.nv);

    // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if( status != Solve_Succeeded ) {
		throw std::runtime_error("Error during initialization of optimization!");
    }

    int pid = 0;
    for (const auto& desiredTransform: desiredTransforms) {
        try {
            mynlp->reset();
            mynlp->set_parameters(z,
                                  model,
                                  desiredTransform,
                                  endT);
        }
        catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
            throw std::runtime_error("Error initializing Ipopt class! Check previous error message!");
        }

        // Run ipopt to solve the optimization problem
        try {
            status = app->OptimizeTNLP(mynlp);
        }
        catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
            throw std::runtime_error("Error solving optimization problem! Check previous error message!");
        }

        // Update initial guess
        solutions.col(pid++) = mynlp->solution;
        z = mynlp->solution;

        // Update feasible flag
        has_optimized &= mynlp->ifFeasible;
    }

    const size_t shape_ptr[] = {model.nv, desiredTransforms.size()};
    auto result = nb::ndarray<nb::numpy, const double>(solutions.data(),
                                                       2,
                                                       shape_ptr,
                                                       nb::handle());
    return nb::make_tuple(result, has_optimized);
}

}; // namespace Kinova
}; // namespace RAPTOR
