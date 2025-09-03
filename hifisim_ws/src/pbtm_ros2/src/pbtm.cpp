/*
* pbtm.cpp
*
* ---------------------------------------------------------------------
* Created by Matthew (matthewoots@gmail.com) in 2022
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#include <pbtm_ros2/pbtm.h>
#include <pbtm_ros2/helper.h>

using namespace helper;

/** @brief Get current uav global nwu pose from Unity */
void pbtm_class::curr_nwu_pose_callback(
	const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	std::lock_guard<std::mutex> pose_lock(pose_mutex);

	_last_pose_time = msg->header.stamp; // keep for stamping if needed
	_last_pose_wall_tp = std::chrono::steady_clock::now();
   	
	// global position in nwu frame
	global_curr_nwu_pose.translation() = Vector3d(
		msg->pose.position.x,
		msg->pose.position.y,
		msg->pose.position.z
	);
	// global rotation in nwu frame
	global_curr_nwu_pose.linear() = Quaterniond(
		msg->pose.orientation.w,
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z).toRotationMatrix();

/* 	printf("curr_nwu_pose_callback = (%f, %f, %f)\n",
		global_curr_nwu_pose.translation().x(),
		global_curr_nwu_pose.translation().y(),
		global_curr_nwu_pose.translation().z()
	);  */
}

/** 
* @brief send_command via mavros to flight controller
* Update pbtm_class::state_command cmd_nwu first and including cmd_nwu.q for yaw command
*/
void pbtm_class::send_command()
{	
	// Eigen::Affine3d global_to_local_setpoint;

    // global_to_local_setpoint.translation() = cmd_nwu.pos;
	// global_to_local_setpoint.linear() = cmd_nwu.q.toRotationMatrix();

	// Eigen::Affine3d enu_cmd_pose = 
    //     global_to_local_t * global_to_local_setpoint;

	// Eigen::Vector3d enu_cmd_vel =
	// 	enu_to_nwu().toRotationMatrix() * cmd_nwu.vel;
	// Eigen::Vector3d enu_cmd_acc =
	// 	enu_to_nwu().toRotationMatrix() * cmd_nwu.acc;

	// mavros_msgs::PositionTarget _cmd;

	// _cmd.header.stamp = node->now();
	// _cmd.position = vector_to_point(enu_cmd_pose.translation());
	// _cmd.velocity = vector_to_ros_vector(enu_cmd_vel);
	// _cmd.acceleration_or_force = vector_to_ros_vector(enu_cmd_acc);

	// Eigen::Vector3d nwu_cmd_euler = euler_rpy(enu_cmd_pose.linear());

	// double cmd_yaw = nwu_cmd_euler.z();
	// _cmd.yaw = (float)constrain_between_180(cmd_yaw);

	Eigen::Vector3d nwu_cmd_euler = euler_rpy(cmd_nwu.q.toRotationMatrix());
	float yaw = (float)constrain_between_180(nwu_cmd_euler.z());

	// removed tmp hardcode yaw to zero: added by Wayne 14-03-2024
	// yaw = 0.0;
	// std::cout << "[pbtm.cpp] yaw = " << yaw << " " << KNRM << std::endl;

	tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, yaw);
	geometry_msgs::msg::PoseStamped global_nwu;
	global_nwu.header.stamp =  _nh->now();
	global_nwu.header.frame_id = "world";
	global_nwu.pose.position.x = cmd_nwu.pos.x();
	global_nwu.pose.position.y = cmd_nwu.pos.y();
	global_nwu.pose.position.z = cmd_nwu.pos.z();
	global_nwu.pose.orientation.x = myQuaternion.getX();
	global_nwu.pose.orientation.y = myQuaternion.getY();
	global_nwu.pose.orientation.z = myQuaternion.getZ();
	global_nwu.pose.orientation.w = myQuaternion.getW();
	_pose_nwu_pub->publish(global_nwu);
}

bool pbtm_class::check_last_time(double tolerance, rclcpp::Time /*time*/)
{
	using namespace std::chrono;
	auto now_tp = steady_clock::now();
	double elapsed = duration_cast<duration<double>>(now_tp - _last_pose_wall_tp).count();
	printf("[%sagent%s pbtm.cpp] %scheck_last_time: Duration = %f, Tolerance = %f%s\n", 
		KGRN, KNRM, KBLU, elapsed, tolerance, KNRM);
	return elapsed < tolerance;
}

void pbtm_class::waypoint_command_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
	std::lock_guard<std::mutex> waypoint_lock(waypoint_command_mutex);

	trajectory_msgs::msg::JointTrajectory copy_msg = *msg;
	std::string _agent_id = copy_msg.joint_names[0];
	// printf("%s\n", _agent_id.c_str());

	if (_agent_id.compare(_id) != 0)
		return;
	
	if (!check_last_time(_timeout, _last_pose_time))
	{
		printf("[%sagent%d%s pbtm.cpp] %sError in rate of local_position msg%s \n", 
			KGRN, uav_id, KNRM, KRED, KNRM);
		return;
	}
	
	int waypoint_command_type = joint_trajectory_to_waypoint(copy_msg);
	
	if (waypoint_command_type >= 0)
	{
		uav_task = waypoint_command_type;
		_setup = false;
	}
	else
		printf("%sagent%d%s rejected invalid mission type! \n",
			KGRN, uav_id, KNRM);
}

void pbtm_class::set_offboard()
{
	//arm_cmd.request.value = true;

    rclcpp::Rate rate(_send_command_rate);
    rclcpp::Time last_request = _nh->now();

    _prev_command_time = _nh->now();

    // *** Make sure takeoff is not immediately sent, this will help to stream the correct data to the program first
    // *** Will give a 1sec buffer ***
    while (_nh->now() - last_request < rclcpp::Duration::from_seconds(1.0))
    {
		// Empty
    }

	home_transformation = global_curr_nwu_pose;

	last_yaw = euler_rpy(home_transformation.linear()).z();
	Eigen::Vector3d current_nwu_pos = home_transformation.translation();
    current_nwu_pos.z() -= 0.05;

	Eigen::Quaterniond q(home_transformation.linear());

	cmd_nwu.pos = current_nwu_pos;
	cmd_nwu.vel = Eigen::Vector3d::Zero();
	cmd_nwu.acc = Eigen::Vector3d::Zero();
	cmd_nwu.q = q;

	// send a few setpoints before starting
	for (int i = 10; rclcpp::ok() && i > 0; --i)
	{
		send_command();
		//rclcpp::spin_some(_nh);
		rate.sleep();
	}
    last_request = _nh->now();

	_offboard_enabled = true;
    printf("[%sagent%d%s pbtm.cpp] %sOffboard mode activated!%s \n", 
		KGRN, uav_id, KNRM, KBLU, KNRM);  

    return;
}

void pbtm_class::agent_timer()
{
	std::lock_guard<std::mutex> waypoint_lock(waypoint_command_mutex);

	// if uav has taken off, the manager cannot takeoff again
	// if (uav_task == kTakeOff && _offboard_enabled)
	// 	uav_task = kHover;

	switch (uav_task)
    {
		case kTakeOff: case kLand:
		{
			if (!_setup)
			{	
				path.poses.clear();

				if (!_offboard_enabled)
					set_offboard();

				initialize_bspline_server(takeoff_land_velocity);

				stime = std::chrono::system_clock::now();
				
				timespan.clear();
				timespan.push_back(0.0);
				timespan.push_back(_duration);
				
				_setup = true;

				printf("[%sagent%d%s pbtm.cpp] kTakeoff/kLand %sFinished setting up bspline!%s \n", 
					KGRN, uav_id, KNRM, KBLU, KNRM); 
				visualize_log_path();
			}

			if (update_get_command_by_time())
				send_command();
			else
			{
				printf("[%sagent%d%s pbtm.cpp] kHover/kIdle \n", 
					KGRN, uav_id, KNRM);

				if (uav_task == kTakeOff)
					uav_task = kHover;
				else if (uav_task == kLand)
					uav_task = kIdle;

				stop_and_hover();
				_setup = false;
			}

		}

		case kHover:
		{
			if (!_offboard_enabled)
			{
				printf("%s[pbtm.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
				break;
			}

			send_command();

			break;
		}

		case kMission: case kHome:
		{
			if (!_offboard_enabled)
			{
				printf("%s[pbtm.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
				break;
			}

			if (!_setup)
			{
				path.poses.clear();
				initialize_bspline_server(_max_velocity);

				stime = std::chrono::system_clock::now();
				
				timespan.clear();
				timespan.push_back(0.0);
				timespan.push_back(_duration);
				
				_setup = true;

				printf("[%sagent%d%s pbtm.cpp] kMission/kHover %sFinished setting up bspline!%s \n", 
					KGRN, uav_id, KNRM, KBLU, KNRM); 
				visualize_log_path();
			}

			if (update_get_command_by_time())
				send_command();
			else
			{
				printf("[%sagent%d%s pbtm.cpp] kHover \n", 
					KGRN, uav_id, KNRM); 
				uav_task = kHover;
				stop_and_hover();
				_setup = false;
			}

			break;
		}

		default:
			break;
		

	}
}

void pbtm_class::initialize_bspline_server(double desired_velocity)
{
	if (wp_pos_vector.empty())
		return;
	
	double total_distance = 0.0;
	for (auto i = 0u; i < wp_pos_vector.size(); i++)
	//for (int i = 0; i < wp_pos_vector.size(); i++)
		total_distance += wp_pos_vector[i].norm();
	
	double est_duration = total_distance / desired_velocity;

	// Re-adjust duration so that our knots and divisions are matching
	double corrected_duration_secs = bsu.get_corrected_duration(
		_send_command_interval, est_duration);

	_knot_size = bsu.get_knots_size(
            _send_command_interval, corrected_duration_secs, _knot_division);

	_knot_interval = corrected_duration_secs / _knot_size;

	control_points.clear();
	control_points = ctt.uniform_distribution_of_cp(
        global_curr_nwu_pose.translation(), wp_pos_vector, 
		_max_velocity, _knot_interval);
	
	// clamp start and also the end to stop uav at the beginning and the end
	for (int i = 0; i < _order; i++)
	{
		control_points.insert(control_points.begin(), 1, control_points[0]);
		control_points.push_back(control_points[control_points.size()-1]);
	}

	_duration = ((double)control_points.size() + (double)_order) * _knot_interval;
}

void pbtm_class::stop_and_hover()
{	
	std::lock_guard<std::mutex> send_command_lock(send_command_mutex);

	// By default cmd pos message would not change;
	// cmd_nwu.pos = cmd_nwu.pos;
	cmd_nwu.vel = Eigen::Vector3d::Zero();
	cmd_nwu.acc = Eigen::Vector3d::Zero();
}

/** 
* @brief joint_trajectory_to_waypoint command and waypoints message translation
* @param points.positions = position waypoint
* @param points.time_from_start = command type 
*/
int pbtm_class::joint_trajectory_to_waypoint(trajectory_msgs::msg::JointTrajectory jt)
{
	// Size of joint trajectory
	int size_of_vector = (int)jt.points.size();
	int mission_type = static_cast<int>(jt.points[0].time_from_start.sec +
                                    jt.points[0].time_from_start.nanosec / 1e9);


	// VehicleTask not within kIdle to kLand
	if (mission_type > 5 || mission_type < 0)
	{
		printf("%sagent%d%s invalid mission_type : %s%d%s! \n", 
			KGRN, uav_id, KNRM, 
			KRED, mission_type, KRED);
		return -1;
	}

	wp_pos_vector.clear();

	if (mission_type == 3)
	{
		for (int i = 0; i < size_of_vector; i++)
		{
			pbtm_class::state_command s;
			if (!jt.points[i].positions.empty())
			{
				double height_setpoint = 
					max(min(jt.points[i].positions[2], height_list[1]), height_list[0]);
				s.pos = Eigen::Vector3d(jt.points[i].positions[0],
						jt.points[i].positions[1], height_setpoint);
			}
			wp_pos_vector.push_back(s.pos);
		}
	}
	else if (mission_type == 1)
	{
		wp_pos_vector.push_back(Eigen::Vector3d(
			global_curr_nwu_pose.translation().x(),
			global_curr_nwu_pose.translation().y(), 
			_takeoff_height));
	}
	else if (mission_type == 4)
	{
		wp_pos_vector.push_back(Eigen::Vector3d(
			home_transformation.translation().x(),
			home_transformation.translation().y(), 
			_takeoff_height));
	}
	else if (mission_type == 5)
	{
		wp_pos_vector.push_back(Eigen::Vector3d(
			home_transformation.translation().x(),
			home_transformation.translation().y(), 
			home_transformation.translation().z()));
	}

	return mission_type;
}

bool pbtm_class::update_get_command_by_time()
{
	// Bspline is updated before we reach here;
	// bs_control_points = cp;

	time_point<std::chrono::system_clock> now_time = 
		system_clock::now();

	double rel_now_time = duration<double>(now_time - stime).count();
	if ((timespan[1] - rel_now_time) < 0)
	{
		std::cout << "[pbtm.cpp] rel_now_time is outside of timespan[1]" << KNRM << std::endl;
		return false;
	}

	bspline_trajectory::bs_pva_state_3d pva3;
	pva3 = bsu.get_single_bspline_3d(
		_order, timespan, control_points, rel_now_time);

	cmd_nwu.pos = pva3.pos[0];
	cmd_nwu.t = rel_now_time;
	
	// if (!pva3.vel.empty())
	cmd_nwu.vel = pva3.vel[0];
	double _norm = sqrt(pow(cmd_nwu.vel.x(),2) + pow(cmd_nwu.vel.y(),2));
	double _norm_x = cmd_nwu.vel.x() / _norm;
	double _norm_y = cmd_nwu.vel.y() / _norm;

	if (!pva3.acc.empty())
		cmd_nwu.acc = pva3.acc[0];
	
	// If velocity is too low, then any noise will cause the yaw to fluctuate
	// Restrict the yaw if velocity is too low
	if (cmd_nwu.vel.norm() >= 0.05)
		last_yaw = atan2(_norm_y,_norm_x);

	cmd_nwu.q = 
		calculate_quadcopter_orientation(cmd_nwu.acc, last_yaw);

	return true;
}

void pbtm_class::visualize_log_path()
{
	path.header.stamp = _nh->now();
	path.header.frame_id = "world";

	for (auto i = 0u; i < control_points.size(); i++)
	//for (int i = 0; i < control_points.size(); i++)
	{
		geometry_msgs::msg::PoseStamped pose;
		pose.header.stamp = _nh->now();
		pose.header.frame_id = "world";
		pose.pose.position = vector_to_point(control_points[i]);
		path.poses.push_back(pose);
	}

	_log_path_pub->publish(path);

}
