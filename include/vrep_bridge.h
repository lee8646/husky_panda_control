#pragma once

#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Dense>

using namespace std;

extern "C" {
#include "extApi.h"
}

const std::string JOINT_HANDLE_PREFIX_1{ "joint_1" };
const std::string JOINT_HANDLE_PREFIX_2{ "joint_2" };
const std::string JOINT_HANDLE_PREFIX_3{ "joint_3" };
const std::string JOINT_HANDLE_PREFIX_4{ "joint_4" };
const std::string JOINT_HANDLE_PREFIX{ "panda_joint" };

class VRepBridge
{
public:

	float wheel_radius;
	float wheel_separation;
	float speed_left;
	float speed_right;

	enum ControlMode { CTRL_VELOCITY };

	VRepBridge(ControlMode mode = CTRL_VELOCITY);
	~VRepBridge();
	
	bool simConnectionCheck();
	void simLoop();

	void write();
	void read();

	void setDesiredVelocity_h(const Eigen::Matrix<double, 4, 1> & desired_velocity);
	void setDesiredPosition_p(const Eigen::Matrix<double, 7, 1> & desired_q);
	void setDesiredTorque_p(const Eigen::Matrix<double, 7, 1> & desired_torque);
	const Eigen::Matrix<double, 4, 1> & getPosition_h();
	const Eigen::Matrix<double, 4, 1> & getVelocity_h();
	const Eigen::Matrix<double, 7, 1> & getPosition_p();
	const Eigen::Matrix<double, 7, 1> & getVelocity_p();
	const size_t getTick() { return tick_; }

private:
	Eigen::Matrix<double, 4, 1> current_pose_;
	Eigen::Matrix<double, 4, 1> current_velocity_;
	Eigen::Matrix<double, 4, 1> desired_velocity_;
	Eigen::Matrix<double, 7, 1> current_q_;
	Eigen::Matrix<double, 7, 1> current_q_dot_;
	Eigen::Matrix<double, 7, 1> desired_q_;
	Eigen::Matrix<double, 7, 1> desired_torque_;

	simxInt clientID_;
	simxInt motorHandle_h_[4];	
	simxInt motorHandle_p_[7];  /// < Depends on simulation envrionment
	simxInt objectHandle_;

	size_t tick_{ 0 };
	
	ControlMode control_mode_;

	void simxErrorCheck(simxInt error);
	void simInit();
	void getHandle(); 	/// < Depends on simulation envrionment
};
