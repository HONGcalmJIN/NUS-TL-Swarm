/*
* pbtm.h
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
#ifndef PBTM_H
#define PBTM_H

#include <string>
#include <mutex>
#include <iostream>
#include <chrono>
#include <ctime>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <random>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
//#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bspline_utils.hpp"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;
using namespace trajectory;

class pbtm_class
{
    private:

        /** @brief VehicleTask determines the task description/mode of the agent **/
        enum VehicleTask
        {
            kIdle,
            kTakeOff,
            kHover,
            kMission,
            kHome,
            kLand
        };

        /** @brief TaskToString interprets the input mission_value **/
        const std::string TaskToString(int v)
        {
            switch (v)
            {
                case kIdle:   return "IDLE";
                case kTakeOff:   return "TAKEOFF";
                case kHover: return "HOVER";
                case kMission:   return "MISSION";
                case kHome:   return "HOME";
                case kLand: return "LAND";
                default:      return "[Unknown Task]";
            }
        }

        /** @brief state_command structure to unify command values to setpoint_raw local **/
        struct state_command
        {
            Eigen::Vector3d pos;
            Eigen::Vector3d vel;
            Eigen::Vector3d acc;
            Eigen::Quaterniond q;
            double t;
        };
        pbtm_class::state_command cmd_nwu;

        /** @brief classes from libbspline packages (functions) that are used in this package **/
        bspline_trajectory bsu;
        common_trajectory_tool ctt;

        rclcpp::Node::SharedPtr _nh;

    	/** @brief Subscribers **/
    	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _nwu_pos_sub;
    	rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _waypoint_sub;

    	/** @brief Publishers **/
    	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_nwu_pub;
    	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _log_path_pub;

        /** @brief Timers **/
        rclcpp::TimerBase::SharedPtr _agent_timer;

        /** @brief Saving rclcpp::Time information of various data **/
        rclcpp::Time _last_pose_time, _prev_command_time;
		/** @brief Wall-clock timestamp of last pose for robust timeout check **/
		std::chrono::steady_clock::time_point _last_pose_wall_tp;

        /** Offboard enabled means that offboard control is active and also uav is armed **/
        bool _offboard_enabled = false;
        bool _setup = false, _state_check = false;
        double takeoff_land_velocity = 0.3;
        int uav_id, uav_task;
        std::string _id;
        double _send_command_interval, _send_command_rate;
        double _timeout, _nwu_yaw_offset, last_yaw, _takeoff_height;
        Eigen::Vector3d _start_global_nwu;

        /** @brief Bspline parameters **/
        int _knot_division, _knot_size;
        double _order, _max_velocity;
        double _knot_interval, _duration;
        time_point<std::chrono::system_clock> stime; // start time for bspline server in time_t
        vector<double> timespan;

        /** @brief mutexes **/
        std::mutex send_command_mutex;
        std::mutex pose_mutex;
        std::mutex waypoint_command_mutex;

        /** @brief Important transformations that handle global and local conversions **/
        Eigen::Affine3d current_transform_enu, global_curr_nwu_pose, home_transformation;
        Eigen::Affine3d global_to_local_t, local_to_global_t;

        /** @brief For path visualization **/
        nav_msgs::msg::Path path;

        vector<Eigen::Vector3d> wp_pos_vector;
        vector<Eigen::Vector3d> control_points;
        vector<double> height_list;

        /** @brief Conversion (rotation) from enu to nwu in the form of w, x, y, z **/
        Quaterniond enu_to_nwu() {return Quaterniond(0.7073883, 0, 0, 0.7068252);}
    
    public:

        /** @brief Constructor **/
        pbtm_class(rclcpp::Node::SharedPtr nodeHandle) : _nh(nodeHandle)
        {
            // Declare and get parameters
            _id = _nh->declare_parameter<std::string>("agent_id", "agent001");
            _send_command_rate = _nh->declare_parameter<double>("send_command_rate", 1.0);
            _timeout = _nh->declare_parameter<double>("timeout", 0.5);
            _takeoff_height = _nh->declare_parameter<double>("takeoff_height", 1.0);

            std::vector<double> position_list(3, 0.0);
            position_list = _nh->declare_parameter<std::vector<double>>("global_start_position", position_list);
            _start_global_nwu.x() = position_list[0];
            _start_global_nwu.y() = position_list[1];
            _start_global_nwu.z() = position_list[2];

            // height min followed by height max
            height_list = _nh->declare_parameter<std::vector<double>>("height_range", std::vector<double>{0.0, 0.0});
            _nwu_yaw_offset = _nh->declare_parameter<double>("yaw_offset_rad", 0.0);

            /** @brief Bspline parameters **/
            _order = _nh->declare_parameter<double>("order", 1.0);
            _max_velocity = _nh->declare_parameter<double>("max_velocity", 1.0);
            _knot_division = _nh->declare_parameter<int>("knot_division", 1);

            _send_command_interval = 1 / _send_command_rate;

			// Initialize wall-clock timestamp to avoid undefined duration before first pose
			_last_pose_wall_tp = std::chrono::steady_clock::now();

            // Reset uav task to idle
            uav_task = 0;
            // Reset commanded state
            cmd_nwu.pos = Eigen::Vector3d::Zero();
            cmd_nwu.vel = Eigen::Vector3d::Zero();
            cmd_nwu.acc = Eigen::Vector3d::Zero();
            cmd_nwu.q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0); 

            // For inclusion of yaw offset
            Eigen::Quaterniond q_corrected;
            q_corrected = AngleAxisd(0, Eigen::Vector3d::UnitX())
                * AngleAxisd(0, Eigen::Vector3d::UnitY())
                * AngleAxisd(_nwu_yaw_offset, Eigen::Vector3d::UnitZ());
            // Setup local_to_global transform
            local_to_global_t = Affine3d::Identity(); 
            local_to_global_t.translate(_start_global_nwu);
            local_to_global_t.rotate(q_corrected.inverse());
            local_to_global_t.rotate(enu_to_nwu().inverse());

            // Setup global_to_local transform
            global_to_local_t = Affine3d::Identity(); 
            global_to_local_t.rotate(enu_to_nwu());
            global_to_local_t.rotate(q_corrected);
            global_to_local_t.translate(-_start_global_nwu);

            /** @brief Get the uav id in int **/
            std::string copy_id = _id; 
            std::string uav_id_char = copy_id.erase(0,5); // removes first 5 character
            uav_id = stoi(uav_id_char);

            /* ------------ Subscribers ------------ */
            _nwu_pos_sub = _nh->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/" + _id + "/global/sim_nwu_pose", 20, 
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->curr_nwu_pose_callback(msg);
                    });

            /** @brief Subscriber that receives waypoint information from user */
            _waypoint_sub = _nh->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "/" + _id + "/trajectory/points", 20, 
                [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
                    this->waypoint_command_callback(msg);
                    });
           
                /* ------------ Publishers ------------ */
            _pose_nwu_pub = _nh->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/" + _id + "/global/nwu_pose", 20);
            _log_path_pub = _nh->create_publisher<nav_msgs::msg::Path>(
                "/" + _id + "/uav/log_path", 10);

            _agent_timer = _nh->create_wall_timer(
                std::chrono::duration<double>(_send_command_interval), 
                std::bind(&pbtm_class::agent_timer, this));

            /* ------------ Service Clients ------------ */
            printf("[%sagent%d%s pbtm.h] global_start_pose [%s%.2lf %.2lf %.2lf%s]! \n", 
                KGRN, uav_id, KNRM,
                KBLU, _start_global_nwu(0), _start_global_nwu(1), _start_global_nwu(2), KNRM);
            printf("[%sagent%d%s pbtm.h] height_range [%s%.2lf %.2lf%s]! \n", 
                KGRN, uav_id, KNRM,
                KBLU, height_list[0], height_list[1], KNRM);
            printf("[%sagent%d%s pbtm.h] yaw_offset_rad [%s%.2lf%s]! \n", 
                KGRN, uav_id, KNRM, KBLU, _nwu_yaw_offset, KNRM);

            printf("%s[agent%d%s pbtm.h] constructed! \n", KGRN, uav_id, KNRM);
        }

        /** @brief Destructor **/
        ~pbtm_class()
        {
            _agent_timer.reset();
        }

        /** @brief check_last_time helps to check whether queried rclcpp::Time satisfy the tolerance margin in the input with the current time **/
        bool check_last_time(double tolerance, rclcpp::Time time);

        /** @brief Main timer (thread) for sending commands and switch modes **/
        void agent_timer();

        /** @brief Pack pbtm_class::state_command cmd_nwu into setpoint_raw/local and publishes it **/
        void send_command();

        /** @brief Initializes the bspline parameters and also sets up the control points **/
        void initialize_bspline_server(double desired_velocity);

        /** @brief To get the command after the Bspline server has started **/
        bool update_get_command_by_time();

        /** @brief Set offboard mode (or tries to set it) **/
        void set_offboard();

        /** @brief stop and hover just sets any last velocity and acceleration command to 0 so that there will be no feedforward **/
        void stop_and_hover();

        /** @brief visualization of the path/trajectory (control points) **/
        void visualize_log_path();

        /** @brief callbacks */
        // void curr_nwu_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr& msg);
        // void waypoint_command_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr& msg);
        void curr_nwu_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void waypoint_command_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

        /** @brief common utility functions */
        int joint_trajectory_to_waypoint(trajectory_msgs::msg::JointTrajectory jt);

};


#endif
