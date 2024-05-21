#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class PosePublisher : public rclcpp::Node {
public:
  PosePublisher() : Node("pose_publisher"), count_(0) {
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("robot/pose", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&PosePublisher::publish_pose, this));
  }

private:
  void publish_pose() {
    auto msg = geometry_msgs::msg::Pose();
    msg.position.x = count_++;
    msg.position.y = count_;
    msg.position.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", msg.position.x);
    publisher_->publish(msg);
  }
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePublisher>());
  rclcpp::shutdown();
  return 0;
}
