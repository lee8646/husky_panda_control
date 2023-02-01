#include "vrep_bridge.h"

VRepBridge::VRepBridge(ControlMode mode)
{
	control_mode_ = mode;
	simInit();
	getHandle();
}
VRepBridge::~VRepBridge()
{
	simxStopSimulation(clientID_, simx_opmode_oneshot_wait);
	simxFinish(clientID_);
}

bool VRepBridge::simConnectionCheck()
{
	return (simxGetConnectionId(clientID_) != -1);
}
void VRepBridge::simLoop()
{
	tick_++;
	simxSynchronousTrigger(clientID_);
}
void VRepBridge::simxErrorCheck(simxInt error)
{
	string errorMsg;
	switch (error)
	{
	case simx_error_noerror:
		return;	// no error
		break;
	case simx_error_timeout_flag:
		errorMsg = "The function timed out (probably the network is down or too slow)";
		break;
	case simx_error_illegal_opmode_flag:
		errorMsg = "The specified operation mode is not supported for the given function";
		break;
	case simx_error_remote_error_flag:
		errorMsg = "The function caused an error on the server side (e.g. an invalid handle was specified)";
		break;
	case simx_error_split_progress_flag:
		errorMsg = "The communication thread is still processing previous split command of the same type";
		break;
	case simx_error_local_error_flag:
		errorMsg = "The function caused an error on the client side";
		break;
	case simx_error_initialize_error_flag:
		errorMsg = "simxStart was not yet called";
		break;
	default:
		errorMsg = "Unknown error.";
		break;
	}

	cout << "[ERROR] An error is occured. code = " << error << endl;
	cout << " - Description" << endl;
	cout << " | " << errorMsg << endl;

	throw std::string(errorMsg);
}

void VRepBridge::simInit()
{
	simxFinish(-1);
	clientID_ = simxStart("127.0.0.1", -3, true, true, 2000, 5);
	if (clientID_ < 0)
	{
		throw std::string("Failed connecting to remote API server. Exiting.");
	}

	simxErrorCheck(simxStartSimulation(clientID_, simx_opmode_oneshot_wait));
	simxErrorCheck(simxSynchronous(clientID_, true));

	cout << "[INFO] V-Rep connection is established." << endl;

}

void VRepBridge::write()
{
	switch (control_mode_)
	{
	case CTRL_VELOCITY:
	{
        wheel_radius = 0.1651;
		wheel_separation = 0.2854*1.875;

        speed_right = desired_velocity_(0) / wheel_radius + desired_velocity_(1) * wheel_separation / wheel_radius;
		speed_left = desired_velocity_(0) / wheel_radius - desired_velocity_(1) * wheel_separation / wheel_radius;

		simxSetJointTargetVelocity(clientID_, motorHandle_h_[0], speed_left, simx_opmode_streaming);
		simxSetJointTargetVelocity(clientID_, motorHandle_h_[1], speed_right, simx_opmode_streaming);
		simxSetJointTargetVelocity(clientID_, motorHandle_h_[2], speed_left, simx_opmode_streaming);
		simxSetJointTargetVelocity(clientID_, motorHandle_h_[3], speed_right, simx_opmode_streaming);
        
		for (size_t i = 0; i < 7; i++)
		{
			simxFloat velocityLimit;

			if (desired_torque_(i) >= 0.0)
				velocityLimit = 10e10f;
			else
				velocityLimit = -10e10f;

			simxSetJointTargetVelocity(clientID_, motorHandle_p_[i], velocityLimit, simx_opmode_streaming);
			simxSetJointForce(clientID_, motorHandle_p_[i], static_cast<float>(abs(desired_torque_(i))), simx_opmode_streaming);

		}
		break;
	}
	}
}

void VRepBridge::read()
{
	for (size_t i = 0; i < 4; i++)
	{
		simxFloat data;
		simxGetJointPosition(clientID_, motorHandle_h_[i], &data, simx_opmode_streaming);
		current_pose_(i) = data;
		simxGetObjectFloatParameter(clientID_, motorHandle_h_[i], 2012, &data, simx_opmode_streaming);
		current_velocity_(i) = data;
	}
	for (size_t i = 0; i < 7; i++)
	{
		simxFloat data;
		simxGetJointPosition(clientID_, motorHandle_p_[i], &data, simx_opmode_streaming);
		current_q_(i) = data;
		simxGetObjectFloatParameter(clientID_, motorHandle_p_[i], 2012, &data, simx_opmode_streaming);
		current_q_dot_(i) = data;
	}
}

void VRepBridge::setDesiredVelocity_h(const Eigen::Matrix<double, 4, 1>& desired_velocity)
{
	desired_velocity_ = desired_velocity;
}

void VRepBridge::setDesiredPosition_p(const Eigen::Matrix<double, 7, 1>& desired_q)
{
	desired_q_ = desired_q;
}

void VRepBridge::setDesiredTorque_p(const Eigen::Matrix<double, 7, 1>& desired_torque)
{
	desired_torque_ = desired_torque;
}

const Eigen::Matrix<double, 4, 1>& VRepBridge::getPosition_h()
{
	return current_pose_;
}

const Eigen::Matrix<double, 4, 1>& VRepBridge::getVelocity_h()
{
	return current_velocity_;
}

const Eigen::Matrix<double, 7, 1>& VRepBridge::getPosition_p()
{
	return current_q_;
}

const Eigen::Matrix<double, 7, 1>& VRepBridge::getVelocity_p()
{
	return current_q_dot_;
}

void VRepBridge::getHandle()

{	
	cout << "[INFO] Getting handles." << endl;
        const string joint_name1 = JOINT_HANDLE_PREFIX_1;
		const string joint_name2 = JOINT_HANDLE_PREFIX_2;
		const string joint_name3 = JOINT_HANDLE_PREFIX_3;
		const string joint_name4 = JOINT_HANDLE_PREFIX_4;
		simxErrorCheck(simxGetObjectHandle(clientID_, joint_name1.c_str(), &motorHandle_h_[0], simx_opmode_oneshot_wait));
		simxErrorCheck(simxGetObjectHandle(clientID_, joint_name2.c_str(), &motorHandle_h_[1], simx_opmode_oneshot_wait));
		simxErrorCheck(simxGetObjectHandle(clientID_, joint_name3.c_str(), &motorHandle_h_[2], simx_opmode_oneshot_wait));
		simxErrorCheck(simxGetObjectHandle(clientID_, joint_name4.c_str(), &motorHandle_h_[3], simx_opmode_oneshot_wait));
    for (int i = 0; i < 7; i++)
	{
		const string joint_name = JOINT_HANDLE_PREFIX + std::to_string(i + 1);
		simxErrorCheck(simxGetObjectHandle(clientID_, joint_name.c_str(), &motorHandle_p_[i], simx_opmode_oneshot_wait));
	}
	cout << "[INFO] The handle has been imported." << endl;
}
