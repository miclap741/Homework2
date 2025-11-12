#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp/rclcpp.hpp"


class KDLController
{

public:

    KDLController(KDLRobot &_robot);    

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);
                           
    Eigen::VectorXd velocity_ctrl_null(KDL::Frame &_xd,
                                       double _Kp);
                           
    Eigen::VectorXd vision_ctrl(const geometry_msgs::msg::PoseStamped& marker_pose, 
    				double _Kp);
    				


private:

    KDLRobot* robot_;
    Eigen::Vector3d cPo_;     		// posizione oggetto nel frame camera
    Eigen::Matrix3d Rc_;       		// rotazione camera
    bool marker_detected_ = false;	// flag ricezione della pose

};

#endif
