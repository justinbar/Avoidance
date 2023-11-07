#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RobotController : public rclcpp::Node
{
public:
  RobotController()
    : Node("control_robot_node"), loop_rate_(1) // a rate of 1 cycle per second
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Node ready.");
  }

  void move()
  {
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.1; // move left 
    for (int i = 1; (i < 3) && rclcpp::ok(); ++i) {
      publisher_->publish(twist_);
      loop_rate_.sleep(); 
      // allow the robot to turn for 5 second
    }
    twist_.angular.z = 0.0;
    publisher_->publish(twist_);

    twist_.linear.x = 0.5;   // Set forward linear velocity 
    //twist_.angular.z = 0.0;  // Set zero angular velocity (no rotation)
    for (int i = 1; (i < 3) && rclcpp::ok(); ++i) {
      publisher_->publish(twist_);
      loop_rate_.sleep();
      // allow the robot to move forward for 5 second
    }

    twist_.linear.x = 0.0;
    //twist_.angular.z=0.0;
    publisher_->publish(twist_);

    RCLCPP_INFO(this->get_logger(), "the robot will stop\n");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist twist_;
  rclcpp::Rate loop_rate_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<RobotController> node = std::make_shared<RobotController>();

  for (int i = 0; (i < 1); ++i) {
    node->move();
    rclcpp::spin_some(node);
  }

  //node->move();
  //rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
