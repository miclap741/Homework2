// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

// parte aggiunta 1.c

#include <functional>
#include <thread>

#include "ros2_kdl_package/action/ros2_kdl_traj.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// #include "action_tutorials_cpp/visibility_control.h"

// fine parte aggiunta 1.c

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

// parte aggiunta 1.c

using Ros2KdlTraj = ros2_kdl_package::action::Ros2KdlTraj;
using GoalHandleRos2KdlTraj = rclcpp_action::ServerGoalHandle<Ros2KdlTraj>;

// fine parte aggiunta 1.c

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub() : Node("ros2_kdl_node"), 
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_),
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // default to "position"
            declare_parameter("ctrl", "velocity"); // default value to "velocity"
            get_parameter("cmd_interface", cmd_interface_);
            get_parameter("ctrl", ctrl_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead...");
                return;
            }
            if (!(ctrl_ == "velocity" || ctrl_ == "velocity_null" ||  ctrl_ == "vision_ctrl"))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'velocity', 'velocity_null' or 'vision_ctrl'          instead...");
                return;
            }

            // declare traj_type parameter (linear, circular)
            declare_parameter("traj_type", "linear");
            get_parameter("traj_type", traj_type_);
            RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
            if (!(traj_type_ == "linear" || traj_type_ == "circular"))
            {
                RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            }

            // declare s_type parameter (trapezoidal, cubic)
            declare_parameter("s_type", "trapezoidal");
            get_parameter("s_type", s_type_);
            RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
            if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected s type is not valid!"); return;
            }

            //Parte aggiunta 

            this->declare_parameter<double>("traj_duration",1.5);
            traj_duration_ = this-> get_parameter("traj_duration").as_double();

            this->declare_parameter<double>("acc_duration",0.5);
            acc_duration_ = this-> get_parameter("acc_duration").as_double();

            this->declare_parameter<double>("total_time",1.5);
            total_time_ = this-> get_parameter("total_time").as_double();

            this->declare_parameter<int>("trajectory_len",150);
            trajectory_len_ = this-> get_parameter("trajectory_len").as_int();

            this->declare_parameter<int>("Kp",5);
            Kp_ = this-> get_parameter("Kp").as_int();
            
            // fine parte aggiunta 

            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();
            
            // inizializzo il controllore
            controller_ = std::make_shared<KDLController>(*robot_);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));
            
            RCLCPP_INFO(this->get_logger(), "Initial pose (KDL): x=%.3f, y=%.3f, z=%.3f",
                init_cart_pose_.p.x(), init_cart_pose_.p.y(), init_cart_pose_.p.z());

            RCLCPP_INFO(this->get_logger(), "Computed init_position (target base): x=%.3f, y=%.3f, z=%.3f",
                init_position[0], init_position[1], init_position[2]);

            // EE's trajectory end position (just opposite y)
            //  
            // end_position << init_position[0], -init_position[1], init_position[2];

            // Parte aggiunta 
            
            this->declare_parameter<double>("end_position.x",init_position[0]); // 
            end_x_ = this-> get_parameter("end_position.x").as_double();

            this->declare_parameter<double>("end_position.y",-init_position[1]); // 
            end_y_ = this-> get_parameter("end_position.y").as_double();

            this->declare_parameter<double>("end_position.z",init_position[2]); // 
            end_z_ = this-> get_parameter("end_position.z").as_double();

            //Definizione variabili dei parametri

            end_position_ << end_x_, end_y_, end_z_;
            
            // Fine parte aggiunta 
            

            // Plan trajectory
            // double traj_duration = 1.5, acc_duration = 0.5, 
	
	    double traj_radius = 0.15;
            // Retrieve the first trajectory point
            if(traj_type_ == "linear"){
                planner_ = KDLPlanner(traj_duration_, acc_duration_, init_position, end_position_); // currently using trapezoidal velocity profile
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.linear_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.linear_traj_cubic(t_);
                }
            } 
            else if(traj_type_ == "circular")
            {
                planner_ = KDLPlanner(traj_duration_, init_position, traj_radius, acc_duration_);
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.circular_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.circular_traj_cubic(t_);
                }
            }
            // // Retrieve the first trajectory point
            // trajectory_point p = planner_.compute_trajectory(t);

            // compute errors
            // Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            // std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }
            
            // parte aggiunta 2b: subscriber aruco
            // Creazione della subscription nel costruttore
            marker_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose",  // topic del marker
                10,                     // queue size
                std::bind(&Iiwa_pub_sub::marker_callback, this, std::placeholders::_1)
            );
            // fine aggiunta

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");

            //parte aggiunta

            using namespace std::placeholders;

            this->action_server_ = rclcpp_action::create_server<Ros2KdlTraj>(
                this,
                "ros2_kdl_traj",
                std::bind(&Iiwa_pub_sub::handle_goal, this, _1, _2),
                std::bind(&Iiwa_pub_sub::handle_cancel, this, _1),
                std::bind(&Iiwa_pub_sub::handle_accepted, this, _1)
            );
            //fine parte aggiunta
        }
    

    private:

        // Variabili aggiunte da noi
        double traj_duration_, acc_duration_, total_time_;
        int trajectory_len_;
        double Kp_;
        std::shared_ptr<KDLController> controller_;

        double end_x_, end_y_, end_z_;
        Eigen::Vector3d  end_position_;
        
        // Subscriber e TF
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;

        // Variabili per salvare la posa del marker
        std::shared_ptr<geometry_msgs::msg::PoseStamped> last_marker_pose_;
        bool marker_available_ = false;

        // Nome del frame della camera
        std::string camera_frame_ = "camera_link";  // Cambialo se diverso nel tuo TF tree
         

        void cmd_publisher(){
		
	    // AGGIUNGI QUESTO CHECK ALL'INIZIO
	    
            if (action_running_) {
         	return;  // Non fare nulla se l'action è in esecuzione
            }
            
            iteration_ = iteration_ + 1;

            // define trajectory
            // double total_time = 1.5; // 
            // int Kp = 5;

            int loop_rate = trajectory_len_ / total_time_;
            double dt = 1.0 / loop_rate;
            t_+=dt;
            
            if (t_ < total_time_){

                // Set endpoint twist
                // double t = iteration_;
                // joint_velocities_.data[2] = 2 * 0.3 * cos(2 * M_PI * t / trajectory_len);
                // joint_velocities_.data[3] = -0.3 * sin(2 * M_PI * t / trajectory_len);

                // Integrate joint velocities
                // joint_positions_.data += joint_velocities_.data * dt;

                // Retrieve the trajectory point based on the trajectory type
                if(traj_type_ == "linear"){
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.linear_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.circular_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.circular_traj_cubic(t_);
                    }
                }

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p_.pos); 

                // compute errors
                Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp_*error))*dt; 

                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity"){
                    if(ctrl_ == "velocity"){
                        // Compute differential IK
                        Vector6d cartvel; cartvel << p_.vel + Kp_*error, o_error;
                        joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    }
                    else if(ctrl_ == "velocity_null"){
                        // Usa il nuovo controllore
                        joint_velocities_cmd_.data = controller_->velocity_ctrl_null(desFrame, Kp_);
                    }
                    else if(ctrl_ == "vision_ctrl"){
                        if (marker_available_) {
                            // Usa il nuovo controllore
                                    // --- DEBUG: stampa la posa usata dal controller ---
                            RCLCPP_INFO(this->get_logger(), "Marker pose in controller: x=%.3f y=%.3f z=%.3f",
                                        last_marker_pose_->pose.position.x,
                                        last_marker_pose_->pose.position.y,
                                        last_marker_pose_->pose.position.z);
                            joint_velocities_cmd_.data = controller_->vision_ctrl(*last_marker_pose_, Kp_);
                            //marker_available_ = false;
                        }else{
                            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                                joint_velocities_cmd_.data[i] = 0.0; //0.0
                            }
                        }
                    }
                }
                else if(cmd_interface_ == "effort"){
                    joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time_);
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                
                // Send joint velocity commands
                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0; //0.0
                    }
                }else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                }
                
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }
        
        // parte aggiunta 2a
        // Dichiarazione della funzione callback
        void marker_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            try {
            
            // --- DEBUG: stampa la posa originale ---
            RCLCPP_INFO(this->get_logger(), "Raw marker pose: x=%.3f y=%.3f z=%.3f",
                        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            
            // Trasforma la posa nel frame della camera
            geometry_msgs::msg::PoseStamped pose_in_cam;
            tf_buffer_.transform(*msg, pose_in_cam, camera_frame_, tf2::durationFromSec(0.05));
            
            RCLCPP_INFO(this->get_logger(), "Transformed pose: x=%.3f y=%.3f z=%.3f",
                        pose_in_cam.pose.position.x, pose_in_cam.pose.position.y, pose_in_cam.pose.position.z);

            // Salva la posa trasformata
            last_marker_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(pose_in_cam);
            
            static auto last_time = this->now();
            auto current_time = this->now();
            RCLCPP_INFO(this->get_logger(), "Marker callback dt: %.3f sec",
                       (current_time - last_time).seconds());
            last_time = current_time;

            
            marker_available_ = true;

            RCLCPP_DEBUG(this->get_logger(),
                         "Marker pose updated in frame: %s (%.3f, %.3f, %.3f)",
                         camera_frame_.c_str(),
                         pose_in_cam.pose.position.x,
                         pose_in_cam.pose.position.y,
                         pose_in_cam.pose.position.z);
            }  
            catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
            }
        }
        // fine aggiunta

        //parte da aggiungere

        rclcpp_action::Server<Ros2KdlTraj>::SharedPtr action_server_; //messa adesso
        
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Ros2KdlTraj::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request", goal-> traj_duration);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleRos2KdlTraj> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }

        void handle_accepted(const std::shared_ptr<GoalHandleRos2KdlTraj> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&Iiwa_pub_sub::execute, this, _1), goal_handle}.detach();
        }



        void execute(const std::shared_ptr<GoalHandleRos2KdlTraj> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            
            // FERMA il timer
            
    	    action_running_ = true;
    
            //rclcpp::Rate loop_hz(1);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Ros2KdlTraj::Feedback>();
            auto result = std::make_shared<Ros2KdlTraj::Result>();

            traj_duration_ = goal->traj_duration;

            // Pianifica la traiettoria
            Eigen::Vector3d start_position(init_cart_pose_.p.data);
            planner_ = KDLPlanner(traj_duration_, acc_duration_, start_position, end_position_);

            t_ = 0.0;
            int loop_hz = trajectory_len_ / total_time_;
            double dt = 1.0 / loop_hz;

            while (t_ < total_time_ && rclcpp::ok()) {
                t_ += dt;

                // Calcola punto di traiettoria
                p_ = planner_.linear_traj_trapezoidal(t_);

                // Ottieni posizione attuale EE
                KDL::Frame cartpos = robot_->getEEFrame();

                // Calcola errore
                Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));

                // Pubblica feedback
                feedback->error = error.norm();

                goal_handle->publish_feedback(feedback);

                // Calcola comando (es. posizione)
                KDL::Frame nextFrame; nextFrame.M = cartpos.M;
                nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp_ * error)) * dt;
                robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);

                for (size_t i = 0; i < joint_positions_cmd_.rows(); ++i) {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }

                // Pubblica comando
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
            }
            
	    // *** AGGIUNGI QUESTO: Aspetta che il robot si stabilizzi ***
	    
	    RCLCPP_INFO(this->get_logger(), "Waiting for robot to stabilize...");
	    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Aspetta 200ms
	    
	    // Aggiorna lo stato del robot prima di calcolare l'errore finale
	    
	    robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            // Check if goal is done
            // Calcola errore finale
            KDL::Frame final_pose = robot_->getEEFrame();
            Eigen::Vector3d final_error = computeLinearError(end_position_, Eigen::Vector3d(final_pose.p.data));

	    // Prima di succeed(), assegna il valore al result
	    result->final_error = final_error.norm();
    
            if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded, %f",result->final_error);
            }
        } 
	
	bool action_running_ = false;
        
        // fine parte aggiunta
        
        
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;

        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        trajectory_point p_;

        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string ctrl_;
        std::string cmd_interface_;
        std::string traj_type_;
        std::string s_type_;

        KDL::Frame init_cart_pose_;
};


int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}

