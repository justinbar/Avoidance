#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "geometry_msgs/msg/twist.hpp"

double front_distance;

class LaserScanSubscriberNode : public rclcpp::Node
{
public:
    LaserScanSubscriberNode() : Node("laser_scan_subscriber"),  loop_rate_(1) 
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
       RCLCPP_INFO(this->get_logger(), "Node ready.");
        // Subscribe to the laser scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::QoS(10),
            std::bind(&LaserScanSubscriberNode::laserScanCallback, this, std::placeholders::_1)
            
        );
    }
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Extract relevant laser scan data
        // Assuming that the LaserScan message already covers the 180-degree range
        // (min_angle = -pi, max_angle = pi), and the samples are evenly spaced.
        int num_samples = msg->ranges.size();
        double angle_increment = (msg->angle_max - msg->angle_min) / (num_samples - 1);
        double front_angle = 0.0; // angle at the front of the robot (180 degrees)

        int front_index = static_cast<int>((front_angle - msg->angle_min) / angle_increment);
        if (front_index >= 0 && front_index < num_samples)
        {
             front_distance = msg->ranges[front_index];
            double front_angle_deg = front_angle * 180 / M_PI;
            RCLCPP_INFO(this->get_logger(), "Front distance: %.2f meters, Front angle: %.2f degrees", front_distance, front_angle_deg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Front scan data not available!");
        }
    }
    void left()
    {
        twist_.linear.x = 0.0;
        twist_.angular.z = 0.1; // move left or right
        for (int i = 1; (i < 3.0) && rclcpp::ok(); ++i) {
        publisher_->publish(twist_);
        loop_rate_.sleep(); 
        // allow the robot to turn for 5 second
        }
        twist_.angular.z = 0.0;
        publisher_->publish(twist_);

        //twist_.angular.z=0.0;
        publisher_->publish(twist_);

        RCLCPP_INFO(this->get_logger(), "the robot will turn left\n");
    }
    void forward()
    {
        twist_.linear.x = 0.1;
        
        
        twist_.angular.z = 0.0;
        publisher_->publish(twist_);

        //twist_.angular.z=0.0;
        publisher_->publish(twist_);

        
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

    //rclcpp::spin(std::make_shared<LaserScanSubscriberNode>());
    
    //node->laserScanCallback();
    while (rclcpp::ok()){
     rclcpp::spin_some(node);
    if( front_distance < 0.4){
     node->left();
     
    }
    else{
        node->forward();
    }
    }
    rclcpp::shutdown();
    return 0;
}
