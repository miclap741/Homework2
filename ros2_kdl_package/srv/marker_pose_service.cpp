#include <rclcpp/rclcpp.hpp>
#include <ros_ign_interfaces/srv/set_entity_pose.hpp>
#include <ros_ign_interfaces/msg/entity.hpp>  
#include "ros2_kdl_package/srv/update_marker_pose.hpp"

using namespace std::chrono_literals;

class MarkerPoseService : public rclcpp::Node
{
public:
  MarkerPoseService() : Node("marker_pose_service")
  {
    // Crea il service server ROS 2
    service_ = this->create_service<ros2_kdl_package::srv::UpdateMarkerPose>(
      "update_marker_pose",
      std::bind(&MarkerPoseService::handle_service, this,
                std::placeholders::_1, std::placeholders::_2));

    // Crea il client verso Ignition
    client_ = this->create_client<ros_ign_interfaces::srv::SetEntityPose>( "/world/aruco_world/set_pose");


    // Aspetta che il servizio Ignition sia disponibile
    while (!client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "In attesa del servizio /set_pose...");
    }
  }

private:
  void handle_service(
    const std::shared_ptr<ros2_kdl_package::srv::UpdateMarkerPose::Request> request,
    std::shared_ptr<ros2_kdl_package::srv::UpdateMarkerPose::Response> response)
  {
    // Prepara la richiesta per Ignition
    auto ign_req = std::make_shared<ros_ign_interfaces::srv::SetEntityPose::Request>();
    ign_req->pose.position.x = request->x;
    ign_req->pose.position.y = request->y;
    ign_req->pose.position.z = request->z;
    ign_req->entity.name = "aruco_marker";
    ign_req->entity.type = ros_ign_interfaces::msg::Entity::MODEL;


    // Invia la richiesta
    auto future = client_->async_send_request(ign_req);

    // Attendi la risposta
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      response->success = true;
      response->message = "Marker aggiornato con successo";
      RCLCPP_INFO(this->get_logger(), "Marker spostato a (%.2f, %.2f, %.2f)",
                  request->x, request->y, request->z);
    } else {
      response->success = false;
      response->message = "Errore nella chiamata a Gazebo";
      RCLCPP_ERROR(this->get_logger(), "Fallita la chiamata al servizio /set_pose");
    }
  }

  rclcpp::Service<ros2_kdl_package::srv::UpdateMarkerPose>::SharedPtr service_;
  rclcpp::Client<ros_ign_interfaces::srv::SetEntityPose>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPoseService>());
  rclcpp::shutdown();
  return 0;
}

