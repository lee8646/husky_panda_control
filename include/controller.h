#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <memory>
#include <fstream>
#include "math_type_define.h"

#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

class WholeBodyController
{   
	// body
	size_t dof_h_;

	Vector4d pose_init_;
	Vector4d velocity_init_;

	Vector4d pose_; 
	Vector4d velocity_;  

	VectorXd velocity_desired_; 
    
	
	Vector3d global_joint_position_[7];

	// arm
	size_t dof_p_;

	// Initial state
	Vector7d q_init_;
	Vector7d qdot_init_;

	// Current state
	Vector7d q_;
	Vector7d qdot_;
	Vector7d qddot_;
	Vector7d torque_;

	// Control value (position controlled)
	Vector7d q_desired_; // Control value
	Vector7d torque_desired_;

	// Dynamics
	Vector7d g_; // Gravity torque
	Matrix7d m_; // Mass matrix
	Matrix7d m_inverse_; // Inverse of mass matrix

	VectorXd q_temp_;	// For RBDL 
	VectorXd qdot_temp_;
	VectorXd qddot_temp_;
	MatrixXd m_temp_;
	VectorXd g_temp_;   // For RBDL 

	VectorXd q_cubic_;
	VectorXd q_target_;

    // for robot model construction
	Math::Vector3d com_position_[7];
	Math::Vector3d joint_posision_[7];

	Model* model_;
	unsigned int body_id_[7];
	unsigned int base_id_; // virtual joint
	unsigned int virtual_body_id_[6];
	Body virtual_body_[6];
	Body base_;
	Body body_[7];
	Joint joint_[7];
	Joint virtual_joint_[6];

	double mass_[7];
	Math::Vector3d axis_[7];
	Math::Vector3d inertia_[7];

	unsigned long tick_;
	double play_time_;
	double hz_;
	double control_start_time_;
	
	std::string control_mode_;
	bool is_mode_changed_;

private:
	void printState();
	void moveWheelVelocity(const VectorXd &target_velocity, double duration);
	void moveJointPositionTorque(const Vector7d &target_position, double duration);

public:
	void readData_h(const VectorXd &position, const VectorXd &velocity);
	void readData_p(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque);
	void readData_p(const Vector7d &position, const Vector7d &velocity);
	const VectorXd & getDesiredVelocity();
	const Vector7d & getDesiredPosition();
	const Vector7d & getDesiredTorque();

public:
		WholeBodyController(double hz) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
	{
			initDimension(); initModel_p();
	}

    void setMode(const std::string & mode);

    void initDimension();
    void initVelocity_h();

	void initModel_p();
    void initPosition_p();

    void compute();
};
