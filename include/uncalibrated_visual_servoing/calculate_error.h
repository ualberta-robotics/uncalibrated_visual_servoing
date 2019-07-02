/* 
 * lpetrich 17/07/18
 */

#ifndef ERROR_CALCULATION_H
#define ERROR_CALCULATION_H

#include "ros/rate.h"
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h" 
#include "std_msgs/UInt32.h" 
#include "uncalibrated_visual_servoing/ErrorVector.h"
#include "uncalibrated_visual_servoing/Point2D.h"
#include "uncalibrated_visual_servoing/PointVector2D.h"
#include "uncalibrated_visual_servoing/utilities.h"
#include <sstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <stdlib.h>
#include <cstring>
#include <unistd.h>


class ErrorCalc 
{
    public:
    	double max;
		bool calculate_now;
		bool stereo_vision;
		bool new_error;
		bool new_error2;
		bool new_eef;
		bool new_eef2;
		Eigen::VectorXd task_ids;
		Eigen::Vector2d end_effector_position;
		Eigen::Vector2d end_effector_position2;
		ros::Publisher pub_error;
		ros::Publisher pub_reset;
		ros::Publisher pub_end_effector_position;

        ErrorCalc(ros::NodeHandle nh_);
        ~ErrorCalc();

        void group_error();
		Eigen::VectorXd normalize_errors(const Eigen::VectorXd& errors, int dimensions);
        std::vector< std::vector<double> > calculate_error(std::vector<Eigen::Vector2d> centers);
        Eigen::VectorXd point_to_point(std::vector<Eigen::Vector2d> sub_vector);
        double point_to_line(std::vector<Eigen::Vector2d> sub_vector);
        Eigen::VectorXd line_to_line(std::vector<Eigen::Vector2d> sub_vector);
        double parallel_lines(std::vector<Eigen::Vector2d> sub_vector);
        double points_to_conic(std::vector<Eigen::Vector2d> sub_vector);
        double point_to_plane(std::vector<Eigen::Vector2d> sub_vector);
		void publish_end_effector();
		void reset();
		void spin();
	private:
		ros::Subscriber sub_trackers;
		ros::Subscriber sub_trackers2;
		ros::Subscriber sub_calculate;
		ros::Subscriber sub_task_ids;
		std::vector< std::vector<double> > current_error_vector;
		std::vector< std::vector<double> > current_error_vector2;

		std::vector< std::vector<double> > get_error() { return current_error_vector; }
		std::vector< std::vector<double> > get_error2() { return current_error_vector2; }

		void callback_centers(const uncalibrated_visual_servoing::PointVector2D::ConstPtr& msg)
		{
			uncalibrated_visual_servoing::PointVector2D c = *msg;
			std::vector<Eigen::Vector2d> v;
		    for (int i = 0; i < c.points.size(); ++i) {
		        Eigen::Vector2d p;
		        p[0] = c.points[i].x;
		        p[1] = c.points[i].y;			        
		        v.push_back(p);
		    }
		    // end_effector_position = v[1];
			current_error_vector = calculate_error(v);
			// new_error = true;
			// new_eef = true;
		}

   //      void callback_centers2(const uvs_control::PointVector2D::ConstPtr& msg)
   //      {
			// if (calculate_now && !new_error2 && task_ids.size() != 0) {
			// 	uvs_control::PointVector2D c2 = *msg;
			// 	std::vector<Eigen::Vector2d> v2;
			//     for (int i = 0; i < c2.points.size(); ++i) {
			//         Eigen::Vector2d p2;
			//         p2[0] = c2.points[i].x;
			//         p2[1] = c2.points[i].y;
			//         v2.push_back(p2);
			//     }
			//     end_effector_position2 = v2[1];
		 //    	current_error_vector2 = calculate_error(v2);
		 //    	new_error2 = true;
		 //    	new_eef2 = true;
			// }
   //      }

   //      void callback_task_ids(const uvs_control::TaskIds::ConstPtr& msg)
   //      {
   //      	uvs_control::TaskIds tasks = *msg;
   //      	Eigen::VectorXd temp(tasks.ids.size());
   //      	for (int i = 0; i < tasks.ids.size(); ++i) {
   //      		temp[i] = tasks.ids[i];
   //      	}
			// task_ids = temp;
   //      }
        
   //      void callback_calculate(const std_msgs::Bool::ConstPtr& msg)
   //      {
			// bool b = msg->data;
			// if (b) { calculate_now = true; }
			// else { reset(); }
   //      }
};


#endif // ERROR_CALCULATION_H