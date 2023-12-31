#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "geometry_msgs/msg/twist.hpp"

double front_distance;
int action;
void timestop();

class LaserScanSubscriberNode : public rclcpp::Node
{
public:
    LaserScanSubscriberNode() : Node("laser_scan_subscriber"), loop_rate_(1)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Node ready.");

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::QoS(10),
            std::bind(&LaserScanSubscriberNode::laserScanCallback, this, std::placeholders::_1)
        );
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int num_samples = msg->ranges.size();
        double angle_increment = (msg->angle_max - msg->angle_min) / (num_samples - 1);
        double front_angle = 0.0; // angle at the front of the robot (180 degrees)

        int front_index = static_cast<int>((front_angle - msg->angle_min) / angle_increment);
        if (front_index >= 0 && front_index < num_samples)
        {
            front_distance = msg->ranges[front_index];
            double front_angle_deg = front_angle * 180 / M_PI;
            RCLCPP_INFO(this->get_logger(), "Front distance: %.2f meters, Front angle: %.2f degrees", front_distance, front_angle_deg);

            // Check if an obstacle is too close
            if (front_distance < 0.5) {
                action = 1; // Set action to avoid the obstacle (turn left)
            } 
            else {
                action = 0; // Set action to move forward
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Front scan data not available!");
        }
    }

    void left()
    {
        timestop();
        twist_.linear.x = 0.0;
        twist_.angular.z = 0.1;
        publisher_->publish(twist_);
        RCLCPP_INFO(this->get_logger(), "Turning left");
    }

    void forward()
    {
        twist_.linear.x = 0.1;
        twist_.angular.z = 0.0;
        publisher_->publish(twist_);
        RCLCPP_INFO(this->get_logger(), "Moving forward");
    }

    void timestop()
    {
        twist_.linear.x = 0.0;
        twist_.angular.z = 0.0;
        for (int i = 1; (i < 2.0) && rclcpp::ok(); ++i) {
        publisher_->publish(twist_);
        loop_rate_.sleep(); 
        // allow the robot to turn for 
        }
        

        publisher_->publish(twist_);

        

        
    }

    void move()
    {
        if (action == 0) {
            forward();
        } else {
            left();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist twist_;
    rclcpp::Rate loop_rate_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LaserScanSubscriberNode> node = std::make_shared<LaserScanSubscriberNode>();

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->move();
    }
    rclcpp::shutdown();
    return 0;
}
