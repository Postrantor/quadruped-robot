#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"

using namespace std::chrono_literals;

class PoseSubscriber : public rclcpp::Node {
public:
  PoseSubscriber() : Node("pose_subscriber") {
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "robot/pose", 10, std::bind(&PoseSubscriber::topic_callback, this, std::placeholders::_1));
    client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
  }

private:
  void topic_callback(const geometry_msgs::msg::Pose::SharedPtr msg) const {
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.pose = *msg;
    request->state.name = "robot";

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto result = client_->async_send_request(request);
  }
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
