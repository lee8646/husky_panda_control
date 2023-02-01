#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>



void WholeBodyController::compute()
{  
    // Kinematics and dynamics calculation ------------------------------
	q_temp_ = q_;
	qdot_temp_ = qdot_;

	RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q_temp_, &qdot_temp_, NULL);
	NonlinearEffects(*model_, q_, Vector7d::Zero(), g_temp_);
	CompositeRigidBodyAlgorithm(*model_, q_, m_temp_, true);

	g_ = g_temp_;
	m_ = m_temp_;

	if (is_mode_changed_)
	{
		is_mode_changed_ = false;

		control_start_time_ = play_time_;

		pose_init_ = pose_;
		velocity_init_ = velocity_;
		q_init_ = q_;
		qdot_init_ = qdot_;
	}
	if (control_mode_ == "go_straight & joint_home")
	{
		Vector2d target_velocity; 
		Vector7d target_position;
		// target_velocity << 1.0, 0.0;
		target_velocity << 1.0, 0.0;
		target_position << 0.0,
		                   0.0,  
						   0.0,   
						   0.0,   
						   0.0,   
						   0.0,   
						   M_PI / 4;
		moveWheelVelocity(target_velocity, 1.0);
		moveJointPositionTorque(target_position, 1.0);
	}
	else if(control_mode_ == "go_backward & joint_move_right")
	{
		Vector2d target_velocity; 
		Vector7d target_position;
		// target_velocity << -1.0, 0.0;
		target_velocity << -1.0, 0.0;
		target_position << M_PI / 2.0, 
		                   0.0,
						   0.0, 
						   -M_PI / 2.0, 
						   0.0,
						   M_PI / 2.0, 
						   M_PI / 2.0;
		moveWheelVelocity(target_velocity, 1.0);
		moveJointPositionTorque(target_position, 1.0);                        
	}
	else if(control_mode_ == "go_straight & joint_move_left")
	{
		Vector2d target_velocity; 
		Vector7d target_position;
		// target_velocity << 0.0, 0.0;
		target_velocity << 0.0, 0.0;
		target_position << -M_PI / 2.0, 
		                   0.0,
						   0.0, 
						   -M_PI / 2.0, 
						   0.0,
						   M_PI / 2.0, 
						   M_PI / 2.0;
		moveWheelVelocity(target_velocity, 1.0);  
		moveJointPositionTorque(target_position, 1.0);   
	}
	else if(control_mode_ == "move_circle & joint_move_left")
	{
		Vector2d target_velocity; 
		Vector7d target_position;
		target_velocity << 1.0, M_PI/4;
		target_position << -M_PI / 2.0, 
		                   0.0,
						   0.0, 
						   -M_PI / 2.0, 
						   0.0,
						   M_PI / 2.0, 
						   M_PI / 2.0;
		moveWheelVelocity(target_velocity, 1.0);  
		moveJointPositionTorque(target_position, 1.0);  
	}
	else if(control_mode_ == "move_circle & joint_move_right")
	{
		Vector2d target_velocity; 
		Vector7d target_position;
		target_velocity << 1.0, M_PI/4;
		target_position << M_PI / 2.0, 
		                   0.0,
						   0.0, 
						   -M_PI / 2.0, 
						   0.0,
						   M_PI / 2.0, 
						   M_PI / 2.0;
		moveWheelVelocity(target_velocity, 1.0);  
		moveJointPositionTorque(target_position, 1.0);  
	}
	else
	{
        Vector2d target_velocity; 
		Vector7d target_position;
		target_velocity << 0.0, 0.0;
		target_position << 0.0, 
		                   -M_PI / 3.0,
						   0.0, 
						   -M_PI / 2.0, 
						   0.0,
						   M_PI / 6.0, 
						   0.0;
		moveWheelVelocity(target_velocity, 1.0);  
		moveJointPositionTorque(target_position, 1.0);  
	}

	printState();
	tick_++;
	play_time_ = tick_ / hz_;
}

void WholeBodyController::printState()
{
	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 50.)
	{
		DBG_CNT = 0;

		cout << "q now    :\t";
		cout << std::fixed << std::setprecision(3) << q_.transpose() << endl;
		cout << "t desired:\t";
		cout << std::fixed << std::setprecision(3) << torque_desired_.transpose() << endl;
	}
}

void WholeBodyController::moveWheelVelocity(const VectorXd & target_velocity, double duration)
{
	velocity_desired_(0) = target_velocity(0);
	velocity_desired_(1) = target_velocity(1);
}

void WholeBodyController::moveJointPositionTorque(const Vector7d &target_position, double duration)
{
	Matrix7d kp, kv;
	Vector7d q_cubic, qd_cubic;
	
	kp = Matrix7d::Identity() * 500.0;
	kv = Matrix7d::Identity() * 20.0;

	for (int i = 0; i < 7; i++)
	{
		qd_cubic(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_position(i), 0, 0);
		q_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_position(i), 0, 0);
	}


	torque_desired_ = m_ * (kp*(q_cubic - q_) + kv*(qd_cubic - qdot_)) + g_;
}

void WholeBodyController::setMode(const std::string & mode)
{
	is_mode_changed_ = true;
	control_mode_ = mode;
	cout << "Current mode (changed) : " << mode << endl;
}

void WholeBodyController::initDimension()
{
	dof_h_ = 4;
	dof_p_ = 7;
	velocity_desired_.setZero();
	q_temp_.resize(7);

	q_desired_.setZero();
	torque_desired_.setZero();

	g_temp_.resize(7);
	m_temp_.resize(7, 7);
}

void WholeBodyController::initModel_p()
{ 
    model_ = new Model();

    model_->gravity = Eigen::Vector3d(0., 0, -GRAVITY);

    // // for floating base //
	// virtual_body_[0] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	// virtual_joint_[0] = Joint(JointTypePrismatic, Eigen::Vector3d::UnitX());
	// virtual_body_[1] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	// virtual_joint_[1] = Joint(JointTypePrismatic, Eigen::Vector3d::UnitY());
	// virtual_body_[2] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	// virtual_joint_[2] = Joint(Math::SpatialVector(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));

	// virtual_body_id_[0] = model_->AddBody(0, Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[0], virtual_body_[0]);
	// virtual_body_id_[1] = model_->AddBody(virtual_body_id_[0], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[1], virtual_body_[1]);


	// double mass = 30.0;
	// base_ = Body(mass, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.5, 0, 0, 0, 100.0, 0, 0, 0, 100.0));
	// base_id_ = model_->AddBody(virtual_body_id_[1], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[2], base_); //body frame = joint frame
	
	// //// for wheel
	// virtual_body_[3] = Body(2.6 *2.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.001, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1));
	// virtual_joint_[3] = Joint(JointTypeRevolute, -Eigen::Vector3d::UnitY());
	// virtual_body_[4] = Body(2.6*2.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.001, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1));
	// virtual_joint_[4] = Joint(JointTypeRevolute, 1.0*Eigen::Vector3d::UnitY());

	// model_->AddBody(base_id_, Math::Xtrans(Math::Vector3d(0.0, 0.25, 0.165)), virtual_joint_[3], virtual_body_[3]);
	// model_->AddBody(base_id_, Math::Xtrans(Math::Vector3d(0.0, -0.25, 0.165)), virtual_joint_[4], virtual_body_[4]);
	// ////////////////////////////////////////////////////////////////////////////////////////////////////
    double mass[7];
    mass[0] = 1.0;
    mass[1] = 1.0;
    mass[2] = 1.0;
    mass[3] = 1.0;
    mass[4] = 1.0;
    mass[5] = 1.0;
    mass[6] = 1.0;

	Vector3d axis[7];
	axis[0] = Eigen::Vector3d::UnitZ();
	axis[1] = Eigen::Vector3d::UnitY();
	axis[2] = Eigen::Vector3d::UnitZ();
	axis[3] = -1.0*Eigen::Vector3d::UnitY();
	axis[4] = Eigen::Vector3d::UnitZ();
	axis[5] = -1.0*Eigen::Vector3d::UnitY();
	axis[6] = -1.0*Eigen::Vector3d::UnitZ();

	Eigen::Vector3d global_joint_position[7];
	global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	global_joint_position[1] = global_joint_position[0];
	global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
	global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
	global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

	// global_joint_position_[0] = Eigen::Vector3d(0.0415, 0.0, 0.7184);
	// global_joint_position_[1] = Eigen::Vector3d(0.0415, 0.0, 0.7184);
	// global_joint_position_[2] = Eigen::Vector3d(0.0415, 0.0, 1.0344);
	// global_joint_position_[3] = Eigen::Vector3d(0.1240, 0.0, 1.0344);
	// global_joint_position_[4] = Eigen::Vector3d(0.0415, 0.0, 1.4184);
	// global_joint_position_[5] = Eigen::Vector3d(0.0415, 0.0, 1.4184);
	// global_joint_position_[6] = Eigen::Vector3d(-0.0465, 0.0, 1.4184);

	joint_posision_[0] = global_joint_position[0];
	for (int i = 1; i < 7; i++)
		joint_posision_[i] = global_joint_position[i] - global_joint_position[i - 1];

	com_position_[0] = Vector3d(0.000096, -0.0346, 0.2575);
	com_position_[1] = Vector3d(0.0002, 0.0344, 0.4094);
	com_position_[2] = Vector3d(0.0334, 0.0266, 0.6076);
	com_position_[3] = Vector3d(0.0331, -0.0266, 0.6914);
	com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
	com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
	com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);

	// com_position_[0] = Vector3d(0.0416, -0.0346, 0.6429);
	// com_position_[1] = Vector3d(0.0417, 0.0345, 0.7948);
	// com_position_[2] = Vector3d(0.0749, 0.0267, 0.9930);
	// com_position_[3] = Vector3d(0.0746, -0.0266, 1.0769);
	// com_position_[4] = Vector3d(0.0428, 0.0424, 1.3097);
	// com_position_[5] = Vector3d(-0.0006, -0.0102, 1.4033);
	// com_position_[6] = Vector3d(-0.0585, -0.0120, 1.4978);


	for (int i = 0; i < 7; i++)
		com_position_[i] -= global_joint_position[i];

    Math::Vector3d inertia[7];
	for (int i = 0; i < 7; i++)
		inertia[i] = Eigen::Vector3d::Identity() * 0.001;

    for (int i = 0; i < 7; i++) {
        body_[i] = Body(mass[i], com_position_[i], inertia[i]);
        joint_[i] = Joint(JointTypeRevolute, axis[i]);
        if (i == 0)
            body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
        else
            body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
    }
}

void WholeBodyController::readData_h(const VectorXd &position, const VectorXd &velocity)
{

		pose_(0) = position(0);
		pose_(1) = position(1);
		pose_(2) = position(2);

		velocity_(0) = velocity(0);
		velocity_(1) = velocity(1);

}

void WholeBodyController::readData_p(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque)
{
	for (size_t i = 0; i < dof_p_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = torque(i);
	}
}
void WholeBodyController::readData_p(const Vector7d &position, const Vector7d &velocity)
{
	for (size_t i = 0; i < dof_p_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = 0;
	}
}

const VectorXd & WholeBodyController::getDesiredVelocity()
{
	return velocity_desired_;
}

const Vector7d & WholeBodyController::getDesiredPosition()
{
	return q_desired_;
}

const Vector7d & WholeBodyController::getDesiredTorque()
{
	return torque_desired_;
}

void WholeBodyController::initVelocity_h()
{
    velocity_init_ = velocity_;
    velocity_desired_ = velocity_init_;
}

void WholeBodyController::initPosition_p()
{
    q_init_ = q_;
    q_desired_ = q_init_;
}
