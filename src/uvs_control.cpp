/*
 * lpetrich 27/06/18
 */
#include <fstream>
#include "uncalibrated_visual_servoing/uvs_control.h"
using namespace std;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

UVSControl::UVSControl(ros::NodeHandle nh_)
{
	dof = 7;
	total_joints = 0;
	image_tol = 100.0;
	default_lambda = 0.15;
	reset = false;
	move_now = false;
	arm = new ArmControl(nh_);
	// Get DOF of arm
	do {
		ROS_INFO_STREAM("Waiting to find robot DOF");
		ros::Duration(1.0).sleep();
		dof = arm->get_dof();
	} while (dof == 0);
	if (dof == 7) {
		total_joints = 7;
		bhand = new BHandControl("/zeus", nh_);
		bhand->set_spread_velocity(25);
		bhand->set_grasp_velocity(60);
	}
	else if (dof == 4) {
		total_joints = 4;
	} else {
		ROS_WARN_STREAM("Invalid DOF, reset and try again");
		exit(EXIT_FAILURE);
	}
	ROS_INFO_STREAM("Robot has " << dof << " DOF");
	error_sub = nh_.subscribe("/user_interface/image_error", 1, &UVSControl::error_cb, this);
	eef_sub = nh_.subscribe("/user_interface/end_effector", 1, &UVSControl::eef_cb, this);
	// reset_sub = nh_.subscribe("/reset", 1, &UVSControl::reset_cb, this);
	// move_sub = nh_.subscribe("/move", 1, &UVSControl::move_cb, this);
	// teleop_sub = nh_.subscribe("/teleop", 1, &UVSControl::teleop_cb, this);
	// joy_sub = nh_.subscribe("/joy", 1, &UVSControl::joy_cb, this);
	// kinematic_model = robot_model_loader.getModel();
	// robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
	// // ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
	// kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
	// kinematic_state->setToDefaultValues();
}

UVSControl::~UVSControl()
{ // shutdown ROS subscribers properly
	error_sub.shutdown();
	eef_sub.shutdown();
	// teleop_sub.shutdown();
}

Eigen::VectorXd UVSControl::calculate_delta_q()
{ // calculates the actual motion change in joint space to use in Broyden's update
	Eigen::VectorXd total_dq;
	Eigen::VectorXd dq(dof);
	Eigen::VectorXd current_joint_positions = arm->get_positions();
	total_dq = current_joint_positions - previous_joint_positions;
	int j = 0;
	for (int i = 0; i < total_joints; ++i) {
		if (active_joints[i]) {
			dq[j] = total_dq[i];
			j++;
		}
	}
	return dq;
}

Eigen::VectorXd UVSControl::calculate_target(const Eigen::VectorXd &current_state, const Eigen::VectorXd &delta)
{ // calculates target vector in joint space to move to with given delta added to active joints
	Eigen::VectorXd target_state(total_joints);
	int j = 0;
	for (int i = 0; i < total_joints; ++i) {
		if (active_joints[i]) {
			target_state[i] = (current_state[i] + delta[j]);
			j++;
		} else {
			target_state[i] = current_state[i];
		}
	}
	return target_state;
}

Eigen::VectorXd UVSControl::calculate_step(const Eigen::VectorXd &current_error_value)
{ // calculates new motion step to take with the control law: step = −λJ+e
	// std::ofstream myfile;
	// myfile.open("predicted_pose.txt", std::ios_base::app);
	Eigen::VectorXd step;
	// Eigen::VectorXd pred_pose;
	Eigen::VectorXd new_joints;
	step = -(jacobian_inverse * current_error_value).transpose();
	new_joints = arm->get_positions() + step;
	// std::cout << new_joints << std::endl;
	// pred_pose = getToolPose(new_joints, total_joints);
	// temp_object_position[0] = pred_pose[0];
	// temp_object_position[1] = pred_pose[1];
	// temp_object_position[2] = pred_pose[2];
	// myfile << pred_pose[0] << ", " << pred_pose[1] << ", " << pred_pose[2] << std::endl;
	// myfile.close();
	step *= 0.1;
	return step;
}

bool UVSControl::convergence_check(const Eigen::VectorXd &current_error)
{ // should we make lambda larger as we get closer to the target? Test
	double pixel_step_size = 30.0;
	double n = current_error.norm();
	Eigen::Vector3d vertical(0, 0, 1);
	// if (n < 300 && !(ready_to_grasp)) {
	// 	bhand->open_grasp();
	// 	bhand->open_spread();
	// 	ready_to_grasp = true;
	// 	std::cout << "ready to grasp object" << std::endl;
	// }
	if (n < image_tol) {
		std::cout << "current error norm is less than image tolerance -- we have arrived at our destination" << std::endl;
		// // for sphere movements
		// object_position = temp_object_position;
		// Eigen::Vector3d tool_position = getToolPosition(arm->get_positions(), total_joints);
		// std::cout << "Object position:" << object_position[0] << ", " << object_position[1] << ", " << object_position[2] << std::endl;
		// spherical_position = cartesian_to_spherical(tool_position - object_position);
		return true;
	}
	lambda = std::max(0.1, pixel_step_size / n);
	if ((1.0 - lambda) * n < image_tol) {
		lambda = 1.0 - image_tol / n;
		std::cout << "Next move places EEF too close to target - adjusting lambda to " << lambda << std::endl;
	}
	std::cout << "current error norm: " << n << std::endl;
	std::cout << "lambda: " << lambda << std::endl;
	return false;
}

bool UVSControl::broyden_update(double alpha)
{ // update jacobian
	Eigen::MatrixXd update(jacobian.rows(), jacobian.cols());
	Eigen::VectorXd current_eef_position;
	Eigen::VectorXd dy;
	Eigen::VectorXd dq;
	double dq_norm;

	dq = calculate_delta_q();
	dq_norm = dq.norm();
	if (dq_norm < 1e-3) {
		cout << "Small dq - no update" << endl;
		std::cout << "dq: \n" << dq.format(CleanFmt) << std::endl;
		return true;
	}
	current_eef_position = get_eef_position();
	dy = current_eef_position - previous_eef_position;
	if (dy.norm() < 5) {
		cout << "Small dy - no update" << endl;
		std::cout << "dy: \n" << dy.format(CleanFmt) << std::endl;
		return true;
	}
	update = ((dy - jacobian * dq) * dq.transpose()) / (dq_norm * dq_norm);
	previous_jacobian = jacobian;
	jacobian = jacobian + (alpha * update);
	if (!pseudoInverse(jacobian, jacobian_inverse)){
		return false;
	}
	std::cout << "current_eef_position: \n" << current_eef_position.format(CleanFmt) << std::endl;
	std::cout << "dq: \n" << dq.format(CleanFmt) << std::endl;
	// std::cout << "dq_norm: \n" << dq_norm << std::endl;
	std::cout << "dy: \n" << dy.format(CleanFmt) << std::endl;
	std::cout << "update: \n" << update.format(CleanFmt) << std::endl;
	std::cout << "jacobian: \n" << jacobian.format(CleanFmt) << std::endl;
	std::cout << "jacobian_inverse: \n" << jacobian_inverse.format(CleanFmt) << std::endl;
	// log(filename, "previous eef position: ", previous_eef_position, false);
	// log(filename, "current eef position: ", current_eef_position, false);
	// log(filename, "dq: ", dq, false);
	// log(filename, "dq norm: ", dq_norm, false);
	// log(filename, "dy: ", dy, false);
	// log(filename, "dy norm: ", dy.norm(), false);
	// log(filename, "broyden update: ", update, false);
	// log(filename, "new jacobian: ", jacobian, false);
	// log(filename, "inverse jacobian: ", jacobian_inverse, false);
	return true;
}

int UVSControl::move_step(bool continous_motion)
{ // step through one loop of VS
	Eigen::VectorXd current_error;
	Eigen::VectorXd current_joint_positions;
	Eigen::VectorXd step_delta;
	Eigen::VectorXd target_position;
	Eigen::VectorXd predicted_times;
	Eigen::VectorXd current_velocity;
	double sleep_time = 1.0;
	// grab and use current error, check for convergence
	current_error = get_error();
	if (convergence_check(current_error)) {
		return 0;
	}
	step_delta = calculate_step(current_error);
	// grab and use current joint positions, check if valid
	current_joint_positions = arm->get_positions();
	target_position = calculate_target(current_joint_positions, step_delta);
	if (!limit_check(target_position, total_joints)) {
		return 1;
	}
	// calculate move run time
	// current_velocity = arm->get_velocities();
	// predicted_times = calculate_rampdown_and_endtime(step_delta, current_velocity);
	// write to screen for debugging
	std::cout << "current_error: \n" << current_error.format(CleanFmt) << std::endl;
	std::cout << "previous_joint_positions: \n" << previous_joint_positions.format(CleanFmt) << std::endl;
	std::cout << "current_joint_positions: \n" << current_joint_positions.format(CleanFmt) << std::endl;
	std::cout << "previous_eef_position: \n" << previous_eef_position.format(CleanFmt) << std::endl;
	std::cout << "step_delta: \n" << step_delta.format(CleanFmt) << std::endl;
	std::cout << "target_position: \n" << target_position.format(CleanFmt) << std::endl;
	// std::cout << "Predicted ramp-down time: " << predicted_times[1] << std::endl;
	// std::cout << "Predicted end time: " << predicted_times[2] << std::endl;
	// save previous state before move
	previous_joint_positions = current_joint_positions;
	previous_eef_position = get_eef_position();
	arm->call_move_joints(target_position, false);
	// check for continuous motion and adjust sleep times accordingly
	// if (continous_motion) {
	// 	// sleep_time = std::max(0.2, predicted_times[1] - 0.05);
	// 	// sleep_time = std::max(0.3, predicted_times[1] - 0.01);
	// 	sleep_time = std::min(1.0, std::max(0.3, (predicted_times[1] + predicted_times[2]) * 0.5)); // range between [0.3, 1.0]
	// } else {
	// 	sleep_time = 1.0;
	// }
	std::cout << "// sleep time: " << sleep_time << std::endl;
	ros::Duration(sleep_time).sleep();
	return 2;
}


int UVSControl::teleop_move_step(bool continous_motion)
{ // step through one loop of VS
	// Eigen::VectorXd current_error;
	// Eigen::VectorXd current_joint_positions;
	// Eigen::VectorXd step_delta(7);
	// Eigen::VectorXd target_position;
	// Eigen::VectorXd predicted_times;
	// Eigen::VectorXd current_velocity;
	// Eigen::MatrixXd motion_vectors;
	// Eigen::VectorXd control_vec(4);
	// double sleep_time;
	// std::cout << "teleop_move_step" << std::endl;
	// current_error = get_error();
	// std::cout << "current_error: " << current_error << std::endl;

	// if (controller_buttons[8])
	// {
	// 	std::cout << "User quit teleop: exiting" << std::endl;
	// 	return 0;
	// }
	// control_vec[0] = (controller_buttons[6] - controller_buttons[7]);
	// control_vec[1] = controller_axes[3];
	// control_vec[2] = -controller_axes[2];
	// if (controller_buttons[9])
	// {
	// 	cout << "Resetting to initial position" << endl;
	// 	goal_joint_angles << 0, 0, 0, 1.8, 0, 0, 0;
	// 	// command_count = 0;
	// 	// start = ros::WallTime::now();
	// 	return 2;
	// }
	// // grab and use current error, check for convergence
	// current_error = get_error();
	// if (convergence_check(current_error))
	// {
	// 	return 0;
	// }
	// step_delta = calculate_step(current_error);
	// motion_vectors = control_plane_vectors(step_delta);
	// // cout << "Motion vectors: " << motion_vectors << endl;
	// step_delta = motion_vectors.col(0) * control_vec[1] + motion_vectors.col(1) * control_vec[2] - step_delta * control_vec[0];
	// // grab and use current joint positions, check if valid
	// current_joint_positions = arm->get_positions();
	// // goal_joint_angles = calculate_target(current_joint_positions, step_delta);
	// goal_joint_angles = current_joint_positions + step_delta;
	// // cout << "Goal angles: " << goal_joint_angles << endl;
	// if (!limit_check(goal_joint_angles, total_joints))
	// {
	// 	return 1;
	// }
	// // calculate move run time
	// // current_velocity = arm->get_velocities();
	// // predicted_times = calculate_rampdown_and_endtime(step_delta, current_velocity);
	// // write to screen for debugging
	// // log(filename, "current_error: ", current_error, false);
	// // log(filename, "previous_joint_positions: ", previous_joint_positions, false);
	// // log(filename, "current_joint_positions: ", current_joint_positions, false);
	// // log(filename, "previous_eef_position: ", previous_eef_position, false);
	// log(filename, "step delta: ", step_delta, false);
	// // log(filename, "target position: ", goal_joint_angles, false);
	// // std::cout << "Predicted ramp-down time: " << predicted_times[1] << std::endl;
	// // std::cout << "Predicted end time: " << predicted_times[2] << std::endl;
	// // save previous state before move
	// previous_joint_positions = current_joint_positions;
	// previous_eef_position = get_eef_position();
	// // arm->call_move_joints(target_position, true);
	// // check for continuous motion and adjust sleep times accordingly
	// // if (continous_motion)
	// // {
	// // 	// sleep_time = std::max(0.2, predicted_times[1] - 0.05);
	// // 	// sleep_time = std::max(0.3, predicted_times[1] - 0.01);
	// // 	sleep_time = std::min(1.0, std::max(0.3, (predicted_times[1] + predicted_times[2]) * 0.5)); // range between [0.3, 1.0]
	// // }
	// // else
	// // {
	// // sleep_time = 1.0;
	// // }
	// // std::cout << "// sleep time: " << sleep_time << std::endl;
	// // ros::Duration(sleep_time).sleep();
	return 2;
}

void UVSControl::teleop_grasp()
{
	// teleop_move = false;
	// int c;
	// int i = 1;
	// std::cout << "\n**************************************" << std::endl;
	// while (true)
	// {
	// 	ros::Rate(10).sleep();
	// 	if (teleop_move)
	// 	{
	// 		std::cout << "received teleop grasp command" << std::endl;
	// 		// teleop_move = false;
	// 		std::cout << "Move: " << i++ << std::endl;
	// 		// ros::Time begin = ros::Time::now();
	// 		c = teleop_grasp_step();
	// 		switch (c)
	// 		{
	// 		case 0: // convergence - return early
	// 			teleop_move = false;
	// 			return;
	// 		// case 1: // joints out of limit, reset jacobian
	// 		// 	jacobian = previous_jacobian;
	// 		// 	log(filename, "target not within joint limits, resetting jacobian to: ", jacobian, false);
	// 		// 	break;
	// 		case 2: // step completed successfully
	// 			teleop_move = false;
	// 			break;
	// 		}
	// 		// std::cout << "loop duration: " << ros::Time::now() - begin << "\n**************************************" << std::endl;
	// 	}
	// }
	return;
}

int UVSControl::teleop_grasp_step()
{
	// Eigen::VectorXd joints;
	// double delta_yaw = M_PI / 40;	  // 45 degrees per second at 30Hz
	// double delta_radians = M_PI / 240; // 5.4 degrees per second at 30Hz
	// Eigen::VectorXd control_vec(4);
	// // std::cout << teleop_direction << std::endl;
	// if (controller_buttons[8])
	// {
	// 	std::cout << "User quit teleop. Exiting.." << std::endl;
	// 	return 0;
	// }
	// if (controller_buttons[3])
	// {
	// 	if (!is_spread)
	// 	{
	// 		std::cout << "Opening spread" << std::endl;
	// 		bhand->open_spread();
	// 		is_spread = true;
	// 		return 2;
	// 	}
	// }
	// if (controller_buttons[0])
	// {
	// 	if (is_spread)
	// 	{
	// 		std::cout << "Closing spread" << std::endl;
	// 		bhand->close_spread();
	// 		is_spread = false;
	// 		return 2;
	// 	}
	// }
	// if (controller_buttons[2])
	// {
	// 	if (grip_closed)
	// 	{
	// 		bhand->open_grasp();
	// 		std::cout << "Opening" << std::endl;
	// 		grip_closed = false;
	// 		return 2;
	// 	}
	// }
	// if (controller_buttons[1])
	// {
	// 	if (!grip_closed)
	// 	{
	// 		bhand->close_grasp();
	// 		std::cout << "Grasping" << std::endl;
	// 		grip_closed = true;
	// 		return 2;
	// 	}
	// }
	// object_position[0] += 0.05 * controller_axes[1];
	// object_position[1] += 0.05 * controller_axes[0]; // 5 cm/s due to weak wrist joint
	// control_vec[0] = (controller_buttons[6] - controller_buttons[7]);
	// control_vec[1] = controller_axes[3];
	// control_vec[2] = -controller_axes[2];
	// control_vec[3] = controller_buttons[5] - controller_buttons[4];

	// if (UVSControl::sphere_move(control_vec))
	// {
	// 	return 2;
	// }
	return 0;
}

bool UVSControl::sphere_move(const Eigen::VectorXd &control_vec)
{
	// const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("arm");
	// const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
	// Eigen::Quaterniond quaternion;
	// geometry_msgs::Pose pose_msg;
	// Eigen::VectorXd ortn(4);
	// Eigen::Vector3d rpy;
	// Eigen::VectorXd full_pose(7);
	// // double delta_radians = 0.017453292519; // one degree
	// double delta_radians = M_PI / 15; // 45 degrees per second at 30Hz
	// Eigen::Vector3d cart_pos;
	// Eigen::Matrix3d rotator;
	// Eigen::Vector3d axis;
	// Eigen::VectorXd current_joint_angles = arm->get_positions();
	// Eigen::VectorXd gains(6);
	// std::vector<double> joints;
	// Eigen::VectorXd goal_joint_positions(7);
	// cart_pos = spherical_to_cartesian(spherical_position);
	// if (std::abs(control_vec[1]) > 0 || std::abs(control_vec[2]) > 0)
	// {
	// 	axis = Eigen::Vector3d::UnitX() * (control_vec[2]) + Eigen::Vector3d::UnitY() * (control_vec[1]);
	// 	axis.normalize();
	// 	rotator = Eigen::AngleAxisd(delta_radians, axis);
	// 	cart_pos = rotator * cart_pos;
	// 	spherical_position = cartesian_to_spherical(cart_pos);
	// }
	// pose_msg = get_pose(object_position, getToolPosition(current_joint_angles, total_joints));
	// bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_msg, "wam/wrist_palm_stump_link", 5, 0.1);
	// if (found_ik)
	// {
	// 	kinematic_state->copyJointGroupPositions(joint_model_group, joints);
	// 	for (std::size_t i = 0; i < joint_names.size(); ++i)
	// 	{
	// 		// ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints[i]);
	// 		goal_joint_positions[i] = joints[i];
	// 	}
	// 	yaw_offset = (current_joint_angles[6] - goal_joint_positions[6]);
	// }
	// else
	// {
	// 	yaw_offset = 0.0;
	// }

	// spherical_position[0] += 0.05 * control_vec[0];
	// yaw_offset -= 2.0 * delta_radians * control_vec[3];
	// std::cout << "Spherical position: \n**************\n"
	// 		  << spherical_position << "\n***********" << std::endl;
	// cart_pos = spherical_to_cartesian(spherical_position);
	// cart_pos = cart_pos + object_position;

	// quaternion = (inwards_normal_to_quaternion(spherical_position));
	// ortn[0] = quaternion.x();
	// ortn[1] = quaternion.y();
	// ortn[2] = quaternion.z();
	// ortn[3] = quaternion.w();
	// full_pose[0] = cart_pos[0];
	// full_pose[1] = cart_pos[1];
	// full_pose[2] = cart_pos[2];
	// full_pose[3] = ortn[0];
	// full_pose[4] = ortn[1];
	// full_pose[5] = ortn[2];
	// full_pose[6] = ortn[3];
	// pose_msg.position.x = cart_pos[0];
	// pose_msg.position.y = cart_pos[1];
	// pose_msg.position.z = cart_pos[2];
	// pose_msg.orientation.x = ortn[0];
	// pose_msg.orientation.y = ortn[1];
	// pose_msg.orientation.z = ortn[2];
	// pose_msg.orientation.w = ortn[3];
	// found_ik = kinematic_state->setFromIK(joint_model_group, pose_msg, "wam/wrist_palm_stump_link", 5, 0.1);
	// if (found_ik)
	// {
	// 	kinematic_state->copyJointGroupPositions(joint_model_group, joints);
	// 	for (std::size_t i = 0; i < joint_names.size(); ++i)
	// 	{
	// 		// ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints[i]);
	// 		goal_joint_positions[i] = joints[i];
	// 	}
	// 	goal_joint_positions[6] += yaw_offset;
	// 	arm->call_move_joints(goal_joint_positions, true);
	// }
	// else
	// {
	// 	ROS_INFO("Did not find IK solution");
	// }
	// std::cout << "Done move" << std::endl;
	return true;
}

void UVSControl::converge(double alpha, int max_iterations, bool continous_motion)
{
	int c;
	std::cout << "\n**************************************" << std::endl;
	for (int i = 0; i < max_iterations; ++i) {
		std::cout << "iteration: " << i << std::endl;
		ros::Time begin = ros::Time::now();
		c = move_step(continous_motion);
		switch (c) {
			case 0: // convergence - return early
				return;
			case 1: // joints out of limit, reset jacobian
				jacobian = previous_jacobian;
				std::cout << "target not within joint limits, resetting jacobian to: \n" << jacobian.format(CleanFmt) << std::endl;
				break;
			case 2: // step completed successfully
				std::cout << "BROYDEN UPDATE:" << std::endl;
				if (!broyden_update(alpha)) { // condition number failed, reset to previous jacobian
					jacobian = previous_jacobian;
					std::cout << "condition number failed, resetting jacobian to: \n" << jacobian.format(CleanFmt) << std::endl;
				}
				break;
		}
		std::cout << "loop duration: " << ros::Time::now() - begin << "\n**************************************" << std::endl;
	}
	return;
}

void UVSControl::teleop_converge(double alpha, int max_iterations, bool continous_motion)
{
	// teleop_move = false;
	// int c;
	// int i = 1;
	// std::cout << "\n**************************************\nWaiting for joystick input for teleoperation" << std::endl;
	// while (true && i < max_iterations)
	// {
	// 	ros::Rate(30).sleep();
	// 	if (teleop_move)
	// 	{
	// 		std::cout << "Received teleop command" << std::endl;
	// 		std::cout << "Step number: " << i++ << std::endl;
	// 		// ros::Time begin = ros::Time::now();
	// 		c = teleop_move_step(continous_motion);
	// 		switch (c)
	// 		{
	// 		case 0: // convergence - return early
	// 			teleop_move = false;
	// 			return;
	// 		case 1: // joints out of limit, reset jacobian
	// 			teleop_move = false;
	// 			jacobian = previous_jacobian;
	// 			log(filename, "target not within joint limits, resetting jacobian to: ", jacobian, false);
	// 			break;
	// 		case 2: // step completed successfully
	// 			arm->call_move_joints(goal_joint_angles, true);
	// 			ros::Duration(0.1).sleep();
	// 			teleop_move = false;
	// 			std::cout << "BROYDEN UPDATE:" << std::endl;
	// 			if (!broyden_update(alpha))
	// 			{ // condition number failed, reset to previous jacobian
	// 				jacobian = previous_jacobian;
	// 				log(filename, "resetting jacobian to: ", jacobian, false);
	// 			}
	// 			break;
	// 		}
	// 		// std::cout << "loop duration: " << ros::Time::now() - begin << "\n**************************************" << std::endl;
	// 	}
	// }
	return;
}

// void UVSControl::set_active_joints()
// { // set which joints to use according to users input
// 	for (int i = 0; i < dof; ++i)
// 	{
// 		active_joints[i] = 0;
// 	}
// 	std::string line;
// 	std::cout << "Which joints would you like to use? >> ";
// 	std::getline(std::cin, line);
// 	std::istringstream ss(line);
// 	int n;
// 	dof = 0;
// 	while (ss >> n)
// 	{
// 		active_joints[n - 1] = 1;
// 		dof += 1;
// 	}
// }

bool UVSControl::jacobian_estimate(double perturbation_delta)
{ // perturb each active joint for the initial jacobian estimation
	Eigen::VectorXd e1;
	Eigen::VectorXd e2;
	Eigen::VectorXd target;
	Eigen::VectorXd position;
	jacobian.resize(get_error().size(), dof);
	initial_jacobian.resize(get_error().size(), dof);
	jacobian_inverse.resize(dof, get_error().size());
	int j = 0;
	for (int i = 0; i < total_joints; ++i) {
		if (active_joints[i]) {
			ros::Duration(0.2).sleep();
			e1 = get_eef_position();
			position = arm->get_positions();
			target = vector_target(position, i, perturbation_delta);
			arm->call_move_joints(target, true);
			ros::Duration(0.2).sleep();
			e2 = get_eef_position();
			ros::Duration(0.2).sleep();
			arm->call_move_joints(position, true);
			jacobian.col(j) = (e2 - e1) / perturbation_delta;
			j++;
		}
	}
	initial_jacobian = jacobian;
	previous_joint_positions = arm->get_positions();
	previous_eef_position = get_eef_position();
	if (!pseudoInverse(jacobian, jacobian_inverse)) {
		std::cout << "Initial jacobian estimate failed -- condition number too large" << std::endl;
		return false;
	}
	std::cout << "initial jacobian: \n" << initial_jacobian.format(CleanFmt) << std::endl;
	std::cout << "inverse jacobian: \n" << jacobian_inverse.format(CleanFmt) << std::endl;

	// log(filename, "initial jacobian: ", initial_jacobian, false);
	// log(filename, "inverse jacobian: ", jacobian_inverse, false);
	return true;
}

Eigen::MatrixXd UVSControl::control_plane_vectors(Eigen::VectorXd &delta_q)
{
	Eigen::MatrixXd control_vectors(total_joints, 3);
	// Eigen::MatrixXd jacobian(3, total_joints);
	// Eigen::MatrixXd jacobian_inv(total_joints, 3);
	// Eigen::VectorXd d_q_1;
	// Eigen::Vector3d d_x;
	// Eigen::Vector3d d_x_1;
	// Eigen::Vector3d d_x_2;
	// Eigen::Vector3d d_x_0;
	// Eigen::Vector3d vertical(0, 0.0, 1.0);
	// Eigen::Vector3d tool_position;
	// Eigen::VectorXd current_positions;
	// current_positions = arm->get_positions();
	// jacobian = arm->get_lin_tool_jacobian();
	// pseudoInverse(jacobian, jacobian_inv);
	// tool_position = getToolPosition(current_positions, total_joints);
	// d_x = getToolPosition(current_positions + delta_q, total_joints) - tool_position;
	// d_x_2 = d_x;
	// d_x_1 = d_x.cross(vertical);
	// d_x_1 *= d_x.norm()/d_x_1.norm();
	// d_x_0 = d_x_1.cross(d_x_2);
	// d_x_0 *= d_x.norm()/d_x_0.norm();

	// control_vectors.col(0) = jacobian_inv * d_x_0;
	// control_vectors.col(1) = jacobian_inv * d_x_1;
	// control_vectors.col(2) = jacobian_inv * d_x_2;
	return control_vectors;
}

Eigen::VectorXd UVSControl::calculate_rampdown_and_endtime(const Eigen::VectorXd &delta, const Eigen::VectorXd &current_velocities)
{ // Code from libbarrett copied here and refactored to determine rampDown time and end time
	// First element of vector is start of plateau (negative is no plateau), second element is ramp down time, last element is predicted end time.
	// cstephens ??/07/2018
	double temp_time_start_down;
	double temp_time_end;
	double temp_time_start_plateau;
	double time_end = -10.0;
	double time_startdown = -10.0;
	double time_start_plateau = -10.0;
	double length;
	double v_init;
	double acc = 0.5;
	double vel = 0.5;
	double v_diff;
	int sign;
	Eigen::VectorXd times(3);
	// for (int i = 0; i < delta.size(); ++i)
	// {
	// 	length = std::abs(delta[i]);
	// 	sign = (int)delta[i] / std::abs(delta[i]);
	// 	v_init = sign * current_velocities[i];
	// 	/* First, is the length too short to decellerate to stop?
	//      * |\
	//      * | \
	//      * |  \__ */
	// 	if ((length < 0.5 * v_init * v_init / acc) && v_init > 0.0)
	// 	{
	// 		/* There are no up or plateau phases */
	// 		time_start_plateau = -1.0;
	// 		temp_time_start_down = 0.0;
	// 		temp_time_end = 2 * length / v_init;
	// 		if (temp_time_start_down > time_startdown)
	// 		{
	// 			time_startdown = temp_time_start_down;
	// 			time_start_plateau = temp_time_start_plateau;
	// 			time_end = temp_time_end;
	// 		}
	// 		continue;
	// 	}
	// 	/* OK, do we not have enough space to plateau?
	// 	 * |
	// 	 * |/\
	// 	 * |  \__ */
	// 	v_diff = vel - v_init;
	// 	if (length < (0.5 * vel * vel + 0.5 * v_diff * v_diff + v_init * v_diff) / acc)
	// 	{
	// 		double v_top;
	// 		v_top = sqrt(length * acc + 0.5 * v_init * v_init);
	// 		temp_time_start_plateau = -1.0;
	// 		temp_time_start_down = (v_top - v_init) / acc;
	// 		temp_time_end = temp_time_start_down + v_top / acc;
	// 		if (temp_time_start_down > time_startdown)
	// 		{
	// 			time_startdown = temp_time_start_down;
	// 			time_start_plateau = temp_time_start_plateau;
	// 			time_end = temp_time_end;
	// 		}
	// 		continue;
	// 	}
	// 	/* OK, we're going to plateau, either up or down
	//      * | __        |\__
	//      * |/  \    or |   \
	//      * |    \__    |    \__ */
	// 	double time_endup = std::abs(v_diff) / acc;
	// 	double s_endup = time_endup * (vel + v_init) / 2;
	// 	/* Compute the ramp down portion */
	// 	double s_startdown = length - 0.5 * vel * vel / acc;
	// 	temp_time_start_down = time_endup + (s_startdown - s_endup) / vel;
	// 	temp_time_end = time_startdown + vel / acc;
	// 	temp_time_start_plateau = time_endup;
	// 	if (temp_time_start_down > time_startdown)
	// 	{
	// 		time_startdown = temp_time_start_down;
	// 		time_start_plateau = temp_time_start_plateau;
	// 		time_end = temp_time_end;
	// 	}
	// }
	// times[0] = time_start_plateau;
	// times[1] = time_startdown;
	// times[2] = time_end;
	return times;
}

void UVSControl::loop()
{ // main loop for user interaction
	bool jacobian_initialized = false;
	bool exit_loop = false;
	bool continous_motion = true;
	double perturbation_delta = 0.0875;
	double alpha = 1.0; // update rate
	int max_iterations = 25;
	double d;
	int c;
	Eigen::VectorXd pose;
	std::string line;
	std::string s;
	lambda = default_lambda; // convergence rate
	while (ros::ok() && !exit_loop) {
		// if (move_now && ready() && jacobian_initialized)
		// {
		// 	converge(alpha, 100, continous_motion);
		// 	lambda = default_lambda;
		// 	move_now = false;
		// }
		std::cout << "************************************************************************************************"
				  << "\nSelect option:"
				  << "\n\tp: Lock joint position"
				  << "\n\tu: Unlock joint position"
				  // << "\n\td: Set Jacobian delta movement (current = " << perturbation_delta << ")"
				  // << "\n\tl: Set step convergence lambda value (current = " << lambda << ")"
				  // << "\n\ta: Set alpha value for broyden update (current = " << alpha << ")"
				  // << "\n\tt: Set image_tol - prevents collision in image space (current = " << image_tol << ")"
				  // << "\n\tc: Set max iteration for convergence (current = " << max_iterations << ")"
				  // << "\n\tm: Set continuous motion (current = " << continous_motion << ")"
				  << "\n\tj: Compute Jacobian"
				  // << "\n\tx: Compute Jacobian with chosen joints"
				  << "\n\tv: Complete VS convergence with set max iterations"
				  << "\n\ts: Compute and move one step"\
  				  // << "\n\tk: Teleop using keyboard node"
				  // << "\n\te: Teleop visual servoing using joy node with set max iterations"
				  // << "\n\tw: Teleop grasp mode using joy node"
				  << "\n\ti: Move to initial position"
				  << "\n\th: Move to home position"
				  << "\n\to: Open grasp"
				  << "\n\tg: Close grasp"
				  << "\n\tz: Close spread"
				  // << "\n\tn: Start new logging file"
				  // << "\n\tf: Print out tool pose"
				  << "\n\tq: quit"
				  << "\n\t>> " << std::endl;
		std::getline(std::cin, line);
		// geometry_msgs::PoseStamped true_ortn;
		// Eigen::VectorXd ortn(4);
		// Eigen::VectorXd gains(6);
		// Eigen::Matrix3d SO_3;
		// Eigen::Vector3d rpy;
		// Eigen::Quaterniond quaternion;
		switch (line[0]) {
		// case 'm':
		// 	s = string_input("Would you like to set motion to continuous or non-continuous?"); // wam_control --> misc_utilities.h
		// 	if (s[0] == 'c' || s[0] == 'C')
		// 	{
		// 		continous_motion = true;
		// 	}
		// 	else
		// 	{
		// 		continous_motion = false;
		// 	}
		// 	std::cout << "continuous motion set to " << BoolToString(continous_motion) << std::endl; // wam_control --> misc_utilities.h
		// 	break;
		case 'p':
			arm->lock_joint_position(true);
			break;
		case 'u':
			// arm->stop_visual_fix();
			arm->lock_joint_position(false);
			break;
		case 'j':
			if (ready()) {
				for (int i = 0; i < dof; ++i) {
					active_joints[i] = 1;
				}
				jacobian_initialized = jacobian_estimate(perturbation_delta);
			}
			break;
		// case 'x':
		// 	if (ready())
		// 	{
		// 		set_active_joints();
		// 		jacobian_initialized = jacobian_estimate(perturbation_delta);
		// 	}
		// 	break;
		case 'i':
			// arm->stop_visual_fix();
			arm->move_to_initial_position();
			bhand->open_grasp();
			bhand->close_spread();
			// grip_closed = false;
			// is_spread = false;
			break;
		// case 'd':
		// 	perturbation_delta = degreesToRadians(double_input(1, 20));
		// 	break;
		// case 'l':
		// 	lambda = double_input(0, 1);
		// 	break;
		// case 'a':
		// 	alpha = double_input(0, 1);
		// 	break;
		// case 'c':
		// 	max_iterations = double_input(0, 500);
		// 	break;
		// case 't':
		// 	image_tol = double_input(0, 500);
		// 	lambda = default_lambda;
		// 	break;
		case 'v':
			if (ready() && jacobian_initialized) {
				converge(alpha, max_iterations - 1, continous_motion);
				lambda = default_lambda;
			} else {
				ROS_WARN_STREAM("Jacobian is not initialized");
			}
			break;
		case 's':
			if (ready() && jacobian_initialized) {
				converge(alpha, 1, false);
				lambda = default_lambda;
			} else {
				ROS_WARN_STREAM("Jacobian is not initialized");
			}
			break;
		// case 'k':
		// 	if (ready() && jacobian_initialized)
		// 	{
		// 		std::cout << "alpha: " << alpha << std::endl;
		// 		std::cout << "max_iterations: " << max_iterations - 1 << std::endl;
		// 		std::cout << "continous_motion: " << continous_motion << std::endl;
		// 		teleop_keyboard_converge(alpha, max_iterations, true);
		// 		lambda = default_lambda;
		// 	}
		// 	else
		// 	{
		// 		ROS_WARN_STREAM("Jacobian is not initialized");
		// 	}
		// 	break;
		// case 'e':
		// 	if (ready() && jacobian_initialized)
		// 	{
		// 		std::cout << "alpha: " << alpha << std::endl;
		// 		std::cout << "max_iterations: " << max_iterations - 1 << std::endl;
		// 		std::cout << "continous_motion: " << continous_motion << std::endl;
		// 		teleop_converge(alpha, max_iterations, true);
		// 		lambda = default_lambda;
		// 	}
		// 	else
		// 	{
		// 		ROS_WARN_STREAM("Jacobian is not initialized");
		// 	}
		// 	break;
		// case 'w':
		// {
		// 	object_position = temp_object_position;
		// 	spherical_position = cartesian_to_spherical(getToolPosition(arm->get_positions(), total_joints) - object_position);
		// 	spherical_position[0] += 0.05;
		// 	teleop_grasp();
		// 	lambda = default_lambda;
		// 	break;
		// }
		case 'h':
			// arm->stop_visual_fix();
			arm->move_to_home_position();
			break;
		case 'o':
			bhand->open_grasp();
			bhand->open_spread();
			break;
		case 'g':
			bhand->close_grasp();
			break;
		case 'z':
			bhand->close_spread();
			break;
		// case 'n':
		// 	filename = prefix + current_time() + ".txt";
		// 	break;
		// case 'f':
		// {
		// 	const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("arm");
		// 	const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
		// 	std::cout << "Made it here" << std::endl;
		// 	Eigen::VectorXd joint_positions = arm->get_positions();
		// 	std::cout << "Made it here too" << std::endl;
		// 	std::vector<double> joints;
		// 	for (int i = 0; i < 7; ++i)
		// 	{
		// 		joints.push_back(joint_positions[i]);
		// 	}
		// 	std::cout << joint_positions << std::endl;
		// 	kinematic_state->setJointGroupPositions(joint_model_group, joints);
		// 	for (std::size_t i = 0; i < joint_names.size(); ++i)
		// 	{
		// 		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints[i]);
		// 	}
		// 	const Eigen::Affine3d &end_effector_state2 = kinematic_state->getGlobalLinkTransform("wam/wrist_palm_stump_link");
		// 	ROS_INFO_STREAM("Translation: " << end_effector_state2.translation());
		// 	// ROS_INFO_STREAM("Rotation: " << end_effector_state2.rotation());
		// 	ROS_INFO_STREAM("Disagreement: " << (end_effector_state2.translation() - getToolPosition(joint_positions, 7)));

		// 	// kinematic_state->setToRandomPositions(joint_model_group);
		// 	// const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("wam/wrist_palm_link");
		// 	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state2, "wam/wrist_palm_stump_link", 10, 0.1);

		// 	// Now, we can print out the IK solution (if found):
		// 	if (found_ik)
		// 	{
		// 		kinematic_state->copyJointGroupPositions(joint_model_group, joints);
		// 		for (std::size_t i = 0; i < joint_names.size(); ++i)
		// 		{
		// 			ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints[i]);
		// 		}
		// 	}
		// 	else
		// 	{
		// 		ROS_INFO("Did not find IK solution");
		// 	}
		// 	break;
		// }

		case 'q':
			exit_loop = true;
			break;
		default:
			ROS_WARN_STREAM("Unknown option");
		}
	}
	// ros::Rate r(60); 
	// while (ros::ok()) {
	// 	Eigen::VectorXd e;
	// 	e = get_error();
	// 	std::cout << "Error: ";
	// 	for (int i = 0; i < e.size(); ++i) {
	// 		std::cout << e[i] << " ";
	// 	}
	// 	e = get_eef_position();
	// 	std::cout << "End Effector: ";
	// 	for (int i = 0; i < e.size(); ++i) {
	// 		std::cout << e[i] << " ";
	// 	}
	// 	std::cout << std::endl;
	// 	ros::spinOnce();
	// 	r.sleep();
	// }
}

void UVSControl::initialize()
{
	if (dof == 7) {
		ROS_INFO_STREAM("Initializing, please wait...");
		while (!arm->move_to_initial_position()) {
			continue;
		}
		ros::Duration(5).sleep();
		while (!bhand->initialize()) {
			continue;
		}
		while (!bhand->close_spread()) {
			continue;
		}
		ros::Duration(10).sleep();
		bhand->hold_finger_position(true);
		ROS_INFO_STREAM("Initialization complete");
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "UVSControl");
	ros::NodeHandle nh_("~");
	ros::AsyncSpinner spinner(0);
	spinner.start();
	UVSControl uvs(nh_);
	// uvs.initialize();
	uvs.loop();
	return 0;
}
