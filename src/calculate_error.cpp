/* 
 * lpetrich 17/07/18
 */

#include "uncalibrated_visual_servoing/calculate_error.h"

ErrorCalc::ErrorCalc(ros::NodeHandle nh_)
{ // error calculator constructor
	max = 1.0e7;
	stereo_vision = false;
	calculate_now = false;
	new_error = false;
	new_error2 = false;
	new_eef = false;
	new_eef2 = false;
	// check how many cameras are connected and setup subscribers/publishers
	// ros::V_string nodes;
	// ros::master::getNodes(nodes);
	// for (int i = 0; i < nodes.size(); i++) {
	// 	std::string prefix = "/cam2";
	// 	if (nodes[i].substr(0, prefix.size()) == prefix) {
	// 		std::cout << "ErrorCalc: Found 2 cameras" << std::endl;
	// 		stereo_vision = true;
	// 		sub_trackers2 = nh_.subscribe("/cam2/trackers/centers", 3, &ErrorCalc::callback_centers2, this);
	// 		break;
	// 	}
	// }
	// if (!stereo_vision) {
	// 	std::cout << "ErrorCalc: Found 1 camera" << std::endl;
	// }
	// sub_task_ids = nh_.subscribe("/task_ids", 3, &ErrorCalc::callback_task_ids, this);
	// sub_calculate = nh_.subscribe("/calculate", 3, &ErrorCalc::callback_calculate, this);
	sub_trackers = nh_.subscribe("/user_interface/tracker_position", 3, &ErrorCalc::callback_centers, this);
	pub_end_effector_position = nh_.advertise<uncalibrated_visual_servoing::PointVector2D>("/eef_pos", 10);
	pub_error = nh_.advertise<uncalibrated_visual_servoing::ErrorVector>("/image_error", 10);
}

ErrorCalc::~ErrorCalc()
{
	sub_task_ids.shutdown();
	sub_calculate.shutdown();
	sub_trackers.shutdown();
	pub_error.shutdown();
	pub_end_effector_position.shutdown();
	if (stereo_vision) {
		sub_trackers2.shutdown();
	}
}

Eigen::VectorXd ErrorCalc::normalize_errors(const Eigen::VectorXd &e, int d)
{ // TODO
	return e;
}

void ErrorCalc::group_error()
{
	uncalibrated_visual_servoing::ErrorVector error_msg;
	std::vector< std::vector<double> > ev = get_error();
	Eigen::VectorXd errors_1D;
	Eigen::VectorXd errors_2D;
	Eigen::VectorXd n1;
	Eigen::VectorXd n2;
	Eigen::VectorXd normalized_error;
	errors_1D = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ev[0].data(), ev[0].size());
	errors_2D = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ev[1].data(), ev[1].size());
	if (stereo_vision) {
		std::vector< std::vector<double> > ev2 = get_error2();
		Eigen::VectorXd errors_1D2;
		Eigen::VectorXd errors_2D2;
		errors_1D2 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ev2[0].data(), ev2[0].size());
		errors_2D2 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ev2[1].data(), ev2[1].size());
		errors_1D = concatenate_vectorxd(errors_1D, errors_1D2);
		errors_2D = concatenate_vectorxd(errors_2D, errors_2D2);
	}
	new_error = false;
	new_error2 = false;
	n1 = normalize_errors(errors_1D, 1);
	n2 = normalize_errors(errors_2D, 2);
	normalized_error = concatenate_vectorxd(n1, n2);
	for (int i = 0; i < normalized_error.size(); ++i) {
		error_msg.error.push_back(normalized_error(i));
	}
	pub_error.publish(error_msg);
	return;
}

std::vector< std::vector<double> > ErrorCalc::calculate_error(std::vector<Eigen::Vector2d> centers)
{ // calculate error according to task id
	std::vector<double> error_1D;
	std::vector<double> error_2D;
	std::vector< std::vector<double> > return_vector;
	int start_idx = 0;
	int skip = 0;
	for (int i = 0; i < task_ids.size(); ++i) {
		Eigen::VectorXd v;
		std::vector<Eigen::Vector2d> sub_vector;
		double e;
		switch ((int)task_ids[i]) {
			case 0:
				skip = 2;
				sub_vector = slice(centers, start_idx, start_idx + skip); // uncalibrated_visual_servoing/utilities.h
				v = point_to_point(sub_vector);
				error_2D.push_back(v(0));
				error_2D.push_back(v(1));
				// e = v.norm();
				break;
			case 1:
				skip = 3;
				sub_vector = slice(centers, start_idx, start_idx + skip);
				e = point_to_line(sub_vector);
				error_1D.push_back(e);
				break;
			case 2:
				skip = 4;
				sub_vector = slice(centers, start_idx, start_idx + skip);
				v = line_to_line(sub_vector);
				error_2D.push_back(v(0));
				error_2D.push_back(v(1));
				// e = v.norm();
				break;
			case 3:
				skip = 4;
				sub_vector = slice(centers, start_idx, start_idx + skip);
				e = parallel_lines(sub_vector);
				error_1D.push_back(e);
				break;
			case 4:
				skip = 0;
				sub_vector = slice(centers, start_idx, start_idx + skip);
				e = points_to_conic(sub_vector);
				error_1D.push_back(e);
				break;
			case 5:
				skip = 0;
				sub_vector = slice(centers, start_idx, start_idx + skip);
				e = point_to_plane(sub_vector);
				break;
			default:
				skip = 0;
				ROS_WARN_STREAM("ErrorCalc: Invalid task id");
				break;
		}
		start_idx += skip;
		//      if (fabs(e) > max) {
		//      	std::cout << fabs(e) << "\n";
		//      	ROS_WARN_STREAM("ErrorCalc: Lost tracker");
		//      	reset();
		// std_msgs::Bool msg;
		// msg.data = true;
		// pub_reset.publish(msg);
		//      }
	}
	return_vector.push_back(error_1D);
	return_vector.push_back(error_2D);
	return return_vector;
}

Eigen::VectorXd ErrorCalc::point_to_point(std::vector<Eigen::Vector2d> v)
{
	Eigen::Vector2d robot;
	Eigen::Vector2d target;
	Eigen::Vector2d result;
	robot = v[0];
	target = v[1];
	// return result.cwiseAbs();
	return target - robot;
}

double ErrorCalc::point_to_line(std::vector<Eigen::Vector2d> v)
{
	Eigen::Vector3d robot, p1, p2;
	robot << v[0](0), v[0](1), 1;
	p1 << v[1](0), v[1](1), 1;
	p2 << v[2](0), v[2](1), 1;
	return p1.cross(p2).dot(robot);
}

Eigen::VectorXd ErrorCalc::line_to_line(std::vector<Eigen::Vector2d> v)
{
	Eigen::Vector3d r1, r2, p1, p2;
	Eigen::Vector2d result(2);
	r1 << v[0](0), v[0](1), 1;
	r2 << v[1](0), v[1](1), 1;
	p1 << v[2](0), v[2](1), 1;
	p2 << v[3](0), v[3](1), 1;
	Eigen::Vector3d line = r1.cross(r2);
	// double result;
	// change to 2 dimensional error
	result << line.dot(p1), line.dot(p2);
	// result = line.dot(p1) + line.dot(p2);
	return result;
}

double ErrorCalc::parallel_lines(std::vector<Eigen::Vector2d> v)
{
	Eigen::Vector3d r1, r2, p1, p2, l1, l2, intersection;
	r1 << v[0](0), v[0](1), 1;
	r2 << v[1](0), v[1](1), 1;
	p1 << v[2](0), v[2](1), 1;
	p2 << v[3](0), v[3](1), 1;
	double result;
	l1 = r1.cross(r2);
	l2 = p1.cross(p2);
	intersection = l1.cross(l2);
	result = intersection(2);
	return result;
}

double ErrorCalc::points_to_conic(std::vector<Eigen::Vector2d> v)
{ // TODO
	// uses code from visp
	// Eigen::MatrixXd A(points_.size() - 1, 5);
	// Eigen::VectorXd b(points_.size() - 1);
	// Eigen::Vector3d p;
	// p << normX(points_[0]->coord.x), normY(points_[0]->coord.y), 1;
	// // A = (y^2 2xy 2x 2y 1)   x = (K0 K1 K2 K3 K4)^T  b = (-x^2 )
	// for(int i = 1; i < points_.size(); i++) {
	//     b(i - 1) = -(normX(points_[i]->coord.x) * normX(points_[i]->coord.x));
	//     A(i - 1, 0) = normY(points_[i]->coord.y) * normY(points_[i]->coord.y);
	//     A(i - 1, 1) = 2 * normX(points_[i]->coord.x) * normY(points_[i]->coord.y);
	//     A(i - 1, 2) = 2 * normX(points_[i]->coord.x);
	//     A(i - 1, 3) = 2 * normY(points_[i]->coord.y);
	//     A(i - 1, 4) = 1;
	// }
	// // Solve Ax = b, least squares minimization.
	// Eigen::JacobiSVD<Eigen::MatrixXd> svd = A.jacobiSvd(Eigen::ComputeThinU |
	//                                                       Eigen::ComputeThinV);
	// e = svd.solve(b); // the ellipse parameters are stored as a class variable. */
	double result = 0.0;
	// // result(0) = (p(0) * p(0)) + (e(0) * p(1) * p(1)) + (2 * e(1) * p(0) * p(1)) + (2 * e(2) * p(0)) + (2 * e(3) * p(1)) + e(4);
	std::cout << "Ellipse Error" << std::endl;
	return result;
}

double ErrorCalc::point_to_plane(std::vector<Eigen::Vector2d> v)
{ // TODO
	Eigen::Vector3d p, p0, vec1, vec2, n, n_unit;
	// p << points_[0]->coord.x, points_[0]->coord.y, 1;
	// p0 << points_[2]->coord.x, points_[2]->coord.y, 1;
	// vec1 << points_[2]->coord.x - points_[1]->coord.x, points_[2]->coord.y - points_[1]->coord.y, 1;
	// vec2 << points_[2]->coord.x - points_[3]->coord.x, points_[2]->coord.y - points_[3]->coord.y, 1;
	double result = 0.0;
	// n = (vec1).cross(vec2);
	// n_unit = (1/n.norm()) * n;
	// double  d = (-(n(0) * p0(0)) - (n(1) * p0(1)) - (n(2) * p0(2)));// / n.norm();
	// //result(0) = n_unit.dot(p) + d;
	// result(0) = (n(0) * p(0)) + (n(1) * p(1)) + (n(0) * p(2)) + d;
	//std::cout << "Plane: " << result(0) << std::endl;
	return result;
}

void ErrorCalc::publish_end_effector()
{ //mono vision eef position publisher
	//cstephens 19/07/18
	uncalibrated_visual_servoing::PointVector2D eef_pts;
	uncalibrated_visual_servoing::Point2D p;
	p.x = end_effector_position(0);
	p.y = end_effector_position(1);
	eef_pts.points.push_back(p);
	if (stereo_vision)
	{
		p.x = end_effector_position2(0);
		p.y = end_effector_position2(1);
		eef_pts.points.push_back(p);
		new_eef2 = false;
	}
	new_eef = false;
	pub_end_effector_position.publish(eef_pts);
	return;
}

void ErrorCalc::reset()
{
	calculate_now = false;
	task_ids.setZero();
	std::cout << "ErrorCalc: resetting\n";
	return;
}

void ErrorCalc::spin()
{
	ros::Rate r(30); // check what works best
	while (ros::ok()) {
		if ((!stereo_vision && new_error) || (stereo_vision && new_error && new_error2)) {
			group_error();
		}
		if ((!stereo_vision && new_eef) || (stereo_vision && new_eef && new_eef2)) {
			publish_end_effector();
		}
		ros::spinOnce();
		r.sleep();
	}
	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "error_calc");
	ros::NodeHandle nh_("~");
	ErrorCalc EC(nh_);
	EC.spin();
	return EXIT_SUCCESS;
}