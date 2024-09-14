#define BOOST_TEST_MODULE ForwardKinematicsTest
#include <boost/test/included/unit_test.hpp>
#include "ForwardKinematics.h"
#include <chrono>

using namespace RAPTOR;

BOOST_AUTO_TEST_SUITE(ForwardKinematicsSuite)
BOOST_AUTO_TEST_CASE(TestForwardKinematicsAccuracy)
{
    // Define robot model
    const std::string urdf_filename = "../Robots/digit-v3/digit-v3-armfixedspecific-floatingbase-springfixed.urdf";
    
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    ForwardKinematicsSolver fkSolver(&model);

    // set joint angles
    std::srand(std::time(nullptr));
    Eigen::VectorXd q = 2 * M_PI * Eigen::VectorXd::Random(model.nq).array() - M_PI;
    pinocchio::forwardKinematics(model, data, q);

    // compute forward kinematics using pinocchio
    // auto start_clock = std::chrono::high_resolution_clock::now();
    // pinocchio::forwardKinematics(model, data, q);
    // auto stop_clock = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_clock - start_clock);
    // std::cout << "Pinocchio FK: " << duration.count() << " nanoseconds" << std::endl;

    // set the start and end joint
    int start = 0;
    int end = model.getJointId("left_toe_B");
    fkSolver.compute(start, end, q);

    // compute forward kinematics using RAPTOR
    // start_clock = std::chrono::high_resolution_clock::now();
    // fkSolver.compute(start, end, q);
    // stop_clock = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_clock - start_clock);
    // std::cout << "RAPTOR FK: " << duration.count() << " nanoseconds" << std::endl;

    // compare the results
    Eigen::Vector3d pinocchio_translation = data.oMi[model.getJointId("left_toe_B")].translation();
    Eigen::Vector3d raptor_translation = fkSolver.getTranslation();

    std::cout << "Pinocchio: " << pinocchio_translation.transpose() << std::endl;
    std::cout << "RAPTOR: " << raptor_translation.transpose() << std::endl;

    BOOST_CHECK_SMALL((pinocchio_translation - raptor_translation).norm(), 1e-10);
}

BOOST_AUTO_TEST_SUITE_END()