/*
 * lpetrich 01/07/18
 */

#ifndef UVS_CONTROL_H
#define UVS_CONTROL_H

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>
#include <boost/timer.hpp>
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "wam_control/misc_utilities.h"
#include "wam_control/arm_control.h"
#include "wam_control/bhand_control.h"
// #include "uncalibrated_visual_servoing/Teleop.h"
// #include "uncalibrated_visual_servoing/Error.h"
// #include "uncalibrated_visual_servoing/TrackPoint.h"
#include "uncalibrated_visual_servoing/PointVector2D.h"
#include "uncalibrated_visual_servoing/utilities.h"

// // MoveIt!
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>

// // Robot state publishing
// #include <moveit/robot_state/conversions.h>
// #include <moveit_msgs/DisplayRobotState.h>

// // Kinematics
// #include <moveit_msgs/GetPositionIK.h>

class UVSControl
{
  public:
	ArmControl *arm;
	BHandControl *bhand;
	bool reset;
	bool move_now;
	bool is_spread = false;
	bool grip_closed = false;
	bool teleop_move;
	int dof;
	int total_joints;
	double image_tol;
	double default_lambda;
	double lambda;
	double control_radius;
	double yaw_offset = 0.0;
	std::string robot_namespace;
	std::string msg;
	std::string prefix;
	std::string filename;
	std::vector<float> controller_axes;
	std::vector<int> controller_buttons;
	std::vector<bool> active_buttons_map = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0};
	std::vector<bool> active_axes_map = {1, 1, 1, 1, 0, 0};
	Eigen::Vector2d teleop_direction;
	Eigen::Vector3d temp_object_position{0.5, 0.0, 0.0};
	Eigen::Vector3d object_position{0.5, 0.0, 0.0};
	Eigen::Vector3d spherical_position{0.3, 0.0, 0.0}; // r, theta, phi
	Eigen::VectorXd previous_eef_position;
	Eigen::VectorXd previous_joint_positions;
	Eigen::MatrixXd previous_jacobian;
	Eigen::MatrixXd initial_jacobian;
	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd jacobian_inverse;
	std::vector<int> active_joints = {1, 1, 1, 1, 1, 1, 1};
	UVSControl(ros::NodeHandle nh);
	~UVSControl();
	Eigen::VectorXd calculate_delta_q();
	Eigen::VectorXd calculate_target(const Eigen::VectorXd &pos, const Eigen::VectorXd &delta);
	Eigen::VectorXd calculate_step(const Eigen::VectorXd &current_error_value);
	Eigen::VectorXd calculate_rampdown_and_endtime(const Eigen::VectorXd &delta, const Eigen::VectorXd &current_velocities);
	Eigen::Matrix<double, 7, 1> goal_joint_angles;
	bool convergence_check(const Eigen::VectorXd &current_error);
	Eigen::MatrixXd control_plane_vectors(Eigen::VectorXd &delta_q);
	void converge(double alpha, int max_iterations, bool continous_motion);
	void teleop_converge(double alpha, int max_iterations, bool continous_motion);
	int move_step(bool continous_motion);
	int teleop_move_step(bool continous_motion);
	int teleop_grasp_step();
	bool comput_IK(geometry_msgs::Pose p);
	bool broyden_update(double alpha);
	bool jacobian_estimate(double perturbation_delta);
	bool sphere_move(const Eigen::VectorXd &control_vec);
	// void set_active_joints();
	void loop();
	void initialize();
	void teleop_grasp();
	// robot_model_loader::RobotModelLoader robot_model_loader;
	// robot_model::RobotModelPtr kinematic_model;
	// robot_state::RobotStatePtr kinematic_state;

  private:
	// Callbacks
	ros::Subscriber error_sub;
	ros::Subscriber eef_sub;
	// ros::Subscriber reset_sub;
	// ros::Subscriber teleop_sub;
	// ros::Subscriber joy_sub;
	// Eigen::VectorXd singular_values;
	Eigen::VectorXd image_error_vector;
	Eigen::VectorXd image_eef_pos;
	bool new_error;
	bool new_eef;

	bool ready()
	{
		if (get_error().size() == 0 || get_eef_position().size() == 0) {
			std::cout << "please initialize trackers" << std::endl;
			return false;
		} else {
			return true;
		}
	}

	Eigen::VectorXd get_error()
	{
		while (!new_error) {
			continue;
		}
		new_error = false;
		return image_error_vector;
	}

	Eigen::VectorXd get_eef_position()
	{
		while (!new_eef) {
			continue;
		} // TODO fix so doesn't get stuck in iteration 6
		new_eef = false;
		return image_eef_pos;
	}

	void error_cb(uncalibrated_visual_servoing::PointVector2D::ConstPtr error)
	{
		uncalibrated_visual_servoing::PointVector2D current_error = *error;
		int sz = current_error.points.size();
		Eigen::VectorXd e(sz * 2);
		int idx = 0;
		for (int i = 0; i < sz; ++i) {
			e[idx++] = current_error.points[i].x;
			e[idx++] = current_error.points[i].y;
		}
		// for (int i = 0; i < e.size(); ++i) {
		// 	std::cout << e[i] << " ";
		// }
		// std::cout << std::endl;
		image_error_vector = e;
		new_error = true;
	}

	void eef_cb(uncalibrated_visual_servoing::PointVector2D::ConstPtr eef)
	{
		uncalibrated_visual_servoing::PointVector2D current_eef = *eef;
		int sz = current_eef.points.size();
		Eigen::VectorXd eef_pos(sz * 2);
		int j = 0;
		for (int i = 0; i < sz; ++i) {
			eef_pos[j++] = current_eef.points[i].x;
			eef_pos[j++] = current_eef.points[i].y;
		}
		image_eef_pos = eef_pos;
		new_eef = true;
	}

	// void reset_cb(std_msgs::Bool data)
	// {
	// 	bool b = data.data;
	// 	if (b)
	// 	{
	// 		reset = true;
	// 	}
	// }

	// // void move_cb(std_msgs::Bool data) {
	// // 	bool b = data.data;
	// // 	if (b) { move_now = true; }
	// // 	else { move_now = false; }
	// // }

	// void teleop_cb(uncalibrated_visual_servoing::Teleop::ConstPtr direction)
	// {
	// 	uncalibrated_visual_servoing::Teleop command = *direction;
	// 	teleop_direction[0] = command.dir_2D[0];
	// 	teleop_direction[1] = command.dir_2D[1];
	// 	std::cout << "Moving in direction: [" << teleop_direction[0] << ", " << teleop_direction[1] << "]" <<std::endl;
	// 	teleop_move = true;
	// }

	// void joy_cb(sensor_msgs::Joy::ConstPtr controllerStatePtr)
	// {
	// 	const std::vector<float> temp_controller_axes = controllerStatePtr->axes;
	// 	const std::vector<int> temp_controller_buttons = controllerStatePtr->buttons;
	// 	const float deadzone = 0.2;
	// 	bool sentinel = false;
	// 	// std::cout << "Callback\n";
	// 	for (int i = 0; i < temp_controller_axes.size(); ++i)
	// 	{
	// 		if (active_axes_map[i])
	// 		{
	// 			// std::cout << i << temp_controller_axes[i] << std::endl;
	// 			if (fabs(temp_controller_axes[i]) > deadzone)
	// 			{
	// 				sentinel = true;
	// 				break;
	// 			}
	// 		}
	// 	}
	// 	if (!sentinel)
	// 	{
	// 		for (int i = 0; i < temp_controller_buttons.size(); ++i)
	// 		{
	// 			if (active_buttons_map[i])
	// 			{
	// 				if (abs(temp_controller_buttons[i]) > 0)
	// 				{
	// 					sentinel = true;
	// 					break;
	// 				}
	// 			}
	// 		}
	// 	}
	// 	if (sentinel)
	// 	{
	// 		controller_axes = temp_controller_axes;
	// 		controller_buttons = temp_controller_buttons;
	// 		teleop_move = true;
	// 		// std::cout << "Here\n";
	// 	}
	// }
};

#endif // UVS_CONTROL_H
