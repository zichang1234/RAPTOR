#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP

#include "Transform.h"

namespace IDTO {

class ForwardKinematicsSolver {
public:
    using Model = pinocchio::Model;
	using Vec3 = Eigen::Vector3d;
	using Mat3 = Eigen::Matrix3d;
    using VecX = Eigen::VectorXd;
	using MatX = Eigen::MatrixXd;

    // Constructor
	ForwardKinematicsSolver() = default;

	ForwardKinematicsSolver(const Model& model_input,
							const Eigen::VectorXi& jtype_input);

    // Destructor
	~ForwardKinematicsSolver() = default;

    // class methods:
		// compute forward kinematics and its high-order derivatives
	void compute(const int start,
				 const int end,
				 const VecX& q, 
				 const Transform* startT = nullptr,
				 const Transform* endT = nullptr,
				 const int order = 0);

	// get the forward kinematics result in different formats
	Vec3 getTranslation() const;

	Mat3 getRotation() const;

	MatX getTranslationJacobian() const;

	void getTranslationHessian(Eigen::Array<MatX, 3, 1>& result) const;
	
    // class members:
	Model model;
	Eigen::VectorXi jtype;

		// a index vector that stores the kinematics chain
	std::vector<int> chain;  

		// results
	Transform T;
	std::vector<Transform> dTdq;
	std::vector<std::vector<Transform>> ddTddq;
	std::vector<std::vector<std::vector<Transform>>> dddTdddq;

	// 	// internal copies
	// int current_order = -1;
	// int end_copy = -1;
	// int start_copy = -1;
	// VecX q_copy;
	// Transform T_copy;
	// std::vector<Transform> dTdq_copy;
	// std::vector<std::vector<Transform>> ddTddq_copy;
	// std::vector<std::vector<std::vector<Transform>>> dddTdddq_copy;
};

}  // namespace IDTO

#endif