#include "kdl_control.h"
#include <Eigen/QR> // per pseudoInverse
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp/rclcpp.hpp"


KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    return Eigen::VectorXd();  // placeholder vuoto per compilare
}

Eigen::VectorXd KDLController::velocity_ctrl_null(KDL::Frame &_xd,
                                                  double _Kp)
{
    // ottieni lo stato del robot
    Eigen::VectorXd q = robot_->getJntValues();		// non serve per la formula
    KDL::Frame x = robot_->getEEFrame();
    Eigen::MatrixXd J = robot_->getEEJacobian().data;
     
    Eigen::MatrixXd J_pos = J.topRows(3);          	// 3 x n: considero solo la parte lineare del Jacobiano
    int n = J.cols();
    
    // calcola l'errore cartesiano
    // _xd.p e x.p sono KDL::Vector
    Eigen::Vector3d _ep;
    _ep << _xd.p[0] - x.p[0],
           _xd.p[1] - x.p[1],
           _xd.p[2] - x.p[2];  // .p restituisce la posizione senza l'orientamento
    
    // calcolo pseudoinversa
    // Si usa la decomposizione ai valori singolari (SVD): J = U * Σ * V^T
    Eigen::MatrixXd J_pinv;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_pos, Eigen::ComputeThinU | Eigen::ComputeThinV);

    double epsilon = 1e-6;  // soglia numerica per stabilità
    Eigen::VectorXd singularValuesInv = svd.singularValues();
    
    // Inversione dei valori singolari
    for (int i = 0; i < singularValuesInv.size(); i++) {
        if (singularValuesInv(i) > epsilon)
            singularValuesInv(i) = 1.0 / singularValuesInv(i);
        else
            singularValuesInv(i) = 0.0;
    }

    // Pseudoinversa: J† = V * Σ† * U^T
    J_pinv = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    
    // calcolo del nullspace:
    // qdot0 (velocità nel null-space per evitare i limiti)
    double lambda = 0.01;   // fattore di scala, regola la forza di "repulsione"
    Eigen::VectorXd _qdot0(n);
    Eigen::MatrixXd q_limits = robot_->getJntLimits();  // col(0)=q_min, col(1)=q_max

    for (int i = 0; i < n; i++) {
        double q_i = q(i);
        double q_min = q_limits(i, 0);
        double q_max = q_limits(i, 1);

        double denom = (q_max - q_i) * (q_i - q_min);
        if (std::abs(denom) > 1e-6)
            _qdot0(i) = lambda * ( (2*q_i - q_max - q_min) / (denom*denom) );
        else
            _qdot0(i) = 0.0;  // evita divisioni instabili vicino ai limiti
    }

    // primo termine
    Eigen::VectorXd v1 = J_pinv * (_Kp * _ep);
    
    // secondo termine, nullspace
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd NullProjector = I - J_pinv * J_pos;		// n x n
    Eigen::VectorXd v2 = NullProjector * _qdot0;      		// n x 1
    
    
    return v1+v2;
}
      
                                                  
Eigen::VectorXd KDLController::vision_ctrl(const geometry_msgs::msg::PoseStamped& marker_pose, double _Kp)
{ 
    // 1) CALCOLO DELLO JACOBIANO DELLA CAMERA
    Eigen::MatrixXd J = robot_->getEEJacobian().data;  // 6 x n
    int n = J.cols();  // numero di giunti

    // 1.1 Trasformazione fissa camera wrt end-effector (dall'URDF)
    Eigen::Vector3d pee_c(0.0, 0.0, -0.005); 

    // 1.2 Rotazione rpy: (0, -1.57, 3.14)
    Eigen::Matrix3d Ree_c;
    Ree_c = Eigen::AngleAxisd(3.14, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitX());

    // 1.3 Calcolo della trasformazione inversa (camera wrt EE)
    Eigen::Matrix3d Rc_ee = Ree_c.transpose();
    Eigen::Vector3d pc_ee = -Rc_ee * pee_c;
    
    Eigen::Vector3d z_cam_check = Ree_c.col(2);
    RCLCPP_INFO(rclcpp::get_logger("kdl_controller"), "Z_cam (wrt EE) = [%f %f %f]", z_cam_check.x(), z_cam_check.y(), z_cam_check.z());

    // 1.4 Matrice skew di pc_ee
    Eigen::Matrix3d S;
    S <<      0, -pc_ee.z(),  pc_ee.y(),
          pc_ee.z(),       0, -pc_ee.x(),
         -pc_ee.y(),  pc_ee.x(),       0;

    // 1.5 Adjoint transformation (6x6)
    Eigen::MatrixXd Ad(6,6);
    Ad.setZero();
    Ad.topLeftCorner(3,3) = Ree_c; //Rc_ee;
    Ad.topRightCorner(3,3) = S * Ree_c; //Rc_ee;
    Ad.bottomRightCorner(3,3) = Ree_c; //Rc_ee;

    // 1.6 Camera Jacobian
    Eigen::MatrixXd Jc = Ad * J;
    
    
    // 2) LETTURA POSIZIONE DEL MARKER
    cPo_ = Eigen::Vector3d(
        marker_pose.pose.position.x,
        marker_pose.pose.position.y,
        marker_pose.pose.position.z
    );
    
    RCLCPP_INFO(rclcpp::get_logger("kdl_controller"), "cPo_: [%f, %f, %f], norm=%f", cPo_.x(), cPo_.y(), cPo_.z(), cPo_.norm());

    // 2.1 Conversione orientamento
    Eigen::Quaterniond q_m(
        marker_pose.pose.orientation.w,
        marker_pose.pose.orientation.x,
        marker_pose.pose.orientation.y,
        marker_pose.pose.orientation.z
    );
    Rc_ = q_m.toRotationMatrix();

    // 2.3 Flag di misura valida
    marker_detected_ = true;
    
    
    // 3) COSTRUZIONE s e suo skew-symmetric
    Eigen::Vector3d s = cPo_.normalized();
    
    Eigen::Matrix3d S_s;
    S_s <<     0, -s.z(),  s.y(),
             s.z(),     0, -s.x(),
            -s.y(),  s.x(),     0;


    // 4) COSTRUZIONE R
    Eigen::MatrixXd R(6,6);
    R.setZero();
    R.topLeftCorner(3,3) = Rc_.transpose();
    R.bottomRightCorner(3,3) = Rc_.transpose();


    // 5) COSTRUZIONE L(s)
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_top = Rc_.transpose();  // rotazione marker -> camera

    Eigen::MatrixXd Ls(3,6);
    Ls.leftCols(3)  = -1.0 / cPo_.norm() * (I - s * s.transpose()) * R_top;
    Ls.rightCols(3) = S_s * R_top;

    // 6) DEFINISCO sd
    Eigen::Vector3d sd(0.0, 0.0, 1.0);  // voglio che l'asse z della camera punti verso il marker
    
    // CONTROLLO SE HO RAGGIUNTO IL TARGET
    Eigen::Vector3d z_cam_target = Rc_.col(2);
    Eigen::Vector3d vec_to_marker = cPo_.normalized();
    double cos_theta = z_cam_target.dot(vec_to_marker);
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);
    double ang_error = std::acos(cos_theta);

    RCLCPP_INFO(rclcpp::get_logger("kdl_controller"), "Angular error: %f rad", ang_error);

    double stop_threshold = 0.05;
    if (ang_error < stop_threshold) {
        Eigen::VectorXd q_dot_zero(n);
        q_dot_zero.setZero();
        return q_dot_zero;
    }
    
    // 7) PRODOTTO Ls*Jc e pseudoinversa
    
    RCLCPP_INFO(rclcpp::get_logger("kdl_controller"), "L(s): %dx%d, Jc: %dx%d", Ls.rows(), Ls.cols(), Jc.rows(), Jc.cols());
    RCLCPP_INFO(rclcpp::get_logger("kdl_controller"), "L(s) max=%f, min=%f", Ls.maxCoeff(), Ls.minCoeff());
    
    Eigen::MatrixXd LJ = Ls * Jc;  // 3 x n
    // Eigen::MatrixXd LJ_pseudo = LJ.completeOrthogonalDecomposition().pseudoInverse();  // n x 3 MA SIAMO SICURI?
    Eigen::MatrixXd LJ_pseudo = (LJ.transpose() * LJ + 1e-6 * Eigen::MatrixXd::Identity(n, n)).inverse() * LJ.transpose();
    RCLCPP_INFO(rclcpp::get_logger("kdl_controller"), "LJ max=%f, min=%f", LJ.maxCoeff(), LJ.minCoeff());
    
    // TERMINE SINISTRO DELLA SOMMA
    Eigen::VectorXd q_dot_main = LJ_pseudo * sd;  // n x 1
    RCLCPP_INFO(rclcpp::get_logger("kdl_controller"), "q_dot_main max=%f, min=%f", q_dot_main.maxCoeff(), q_dot_main.minCoeff());

    // 8) CALCOLO DEL NULL-SPACE:
    Eigen::VectorXd q = robot_->getJntValues();	
    // qdot0 (velocità nel null-space per evitare i limiti)
    double lambda = 0.01;   // fattore di scala, regola la forza di "repulsione"
    Eigen::VectorXd _qdot0(n);
    Eigen::MatrixXd q_limits = robot_->getJntLimits();  // col(0)=q_min, col(1)=q_max

    for (int i = 0; i < n; i++) {
        double q_i = q(i);
        double q_min = q_limits(i, 0);
        double q_max = q_limits(i, 1);

        double denom = (q_max - q_i) * (q_i - q_min);
        if (std::abs(denom) > 1e-6)
            _qdot0(i) = lambda * ( (2*q_i - q_max - q_min) / (denom*denom) );
        else
            _qdot0(i) = 0.0;  // evita divisioni instabili vicino ai limiti
    }
    
    // 9) CALCOLO DI N
    RCLCPP_INFO(rclcpp::get_logger("kdl_controller"), "q_dot0 max=%f, min=%f", _qdot0.maxCoeff(), _qdot0.minCoeff());
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - LJ_pseudo * LJ;
    
    // CALCOLO DELLA LEGGE DI CONTROLLO FINALE
    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(n, n) * _Kp;	// Matrice K di gain
    Eigen::VectorXd q_dot = K * q_dot_main + N * _qdot0;
    RCLCPP_INFO(rclcpp::get_logger("kdl_controller"), "q_dot max=%f, min=%f", q_dot.maxCoeff(), q_dot.minCoeff());

    return q_dot;
}






