#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

double front_distance;
double left_distance;
double right_distance;
void timestop();
int action;
int count;



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
        double front_angle = 0.0; // angle at the front of the robot (0 degrees)
        double left_angle = M_PI/2; // angle at the front of the robot (90 degrees)
        double right_angle = -M_PI/2; // angle at the front of the robot (270 degrees or -90 deg)

        int front_index = static_cast<int>((front_angle - msg->angle_min) / angle_increment);
        int left_index = static_cast<int>((left_angle - msg->angle_min) / angle_increment);
        int right_index = static_cast<int>((right_angle - msg->angle_min) / angle_increment);
        if (front_index >= 0 && front_index < num_samples)
        {
             front_distance = msg->ranges[front_index];
             left_distance = msg->ranges[left_index];
             right_distance = msg->ranges[right_index];
            double front_angle_deg = front_angle * 180 / M_PI;
            double left_angle_deg = left_angle * 180 / M_PI;
            double right_angle_deg = right_angle * 180 / M_PI;
            RCLCPP_INFO(this->get_logger(), "Front distance: %.2f meters, Front angle: %.2f degrees", front_distance, front_angle_deg);
            RCLCPP_INFO(this->get_logger(), "left distance: %.2f meters, left angle: %.2f degrees", left_distance, left_angle_deg);
            RCLCPP_INFO(this->get_logger(), "right distance: %.2f meters, right angle: %.2f degrees", right_distance, right_angle_deg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "laser data not available!");
        }
    }
    void left()
    {
        timestop();
        twist_.linear.x = 0.0;
        twist_.angular.z = -0.1; // move left 
        for (int i = 1; (i < 3.0) && rclcpp::ok(); ++i) {
        publisher_->publish(twist_);
        loop_rate_.sleep(); 
        // allow the robot to turn
        }
        twist_.angular.z = 0.0;
        publisher_->publish(twist_);
        timestop();

        

        RCLCPP_INFO(this->get_logger(), "the robot will turn left\n");
    }
    void right()
    {
        timestop();
        twist_.linear.x = 0.0;
        twist_.angular.z = 0.1; // move right
        //count = 0; // Reset count
        for (int i = 1; (i < 3.0) && rclcpp::ok(); ++i) {
        publisher_->publish(twist_);
        loop_rate_.sleep(); 
        // allow the robot to turn for 
        }
        publisher_->publish(twist_);

       timestop(); 

        //RCLCPP_INFO(this->get_logger(), "the robot will turn right\n");
    }
    void forward()
    {
        twist_.linear.x = 0.1;
        
        
        twist_.angular.z = 0.0;
        

        for (int i = 1; (i < 2.0) && rclcpp::ok(); ++i) {
        publisher_->publish(twist_);
        loop_rate_.sleep(); 
        // allow the robot to turn for 
        }
        publisher_->publish(twist_);
        //clcpp::sleep_for(std::chrono::seconds(1)); //delay

        
        
    }
    void timestop()
    {
        twist_.linear.x = 0.0;
        twist_.angular.z = 0.0;
        for (int i = 1; (i < 4.0) && rclcpp::ok(); ++i) {
        publisher_->publish(twist_);
        loop_rate_.sleep(); 
        // allow the robot to turn for 
        }
        

        publisher_->publish(twist_);

        

        
    }
     void stop()
    {
        twist_.linear.x = 0.0;
        twist_.angular.z = 0.0;
        
        

        publisher_->publish(twist_);

        

        
    }
    void move()
    {
     if( front_distance < 0.75 ){
      if(left_distance > .55){
        action = 0;
      }
      else if(right_distance > .55){
        action = 1;
      }
      else{
        action = 2;
      }
     
     }
     else{
        action= 3;
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

    //rclcpp::spin(std::make_shared<LaserScanSubscriberNode>());
    
    //node->laserScanCallback();
    while (rclcpp::ok()){
     rclcpp::spin_some(node);
     node->move();

        switch (action) {
                
                case 0: // Fixed syntax issue here (removed semicolon)
                    node->left();
                    break;
                    
                case 1: // Fixed syntax issue here (removed semicolon)
                    //node->stop();
                    node->right();
                    break;
                case 2: // Fixed syntax issue here (removed semicolon)
                    //node->stop();
                    node->stop();
                    break;
                case 3: // Fixed syntax issue here (removed semicolon)
                    //node->stop();
                    node->forward();
                    break;
                default:
                    node->forward();
                    break;
            }

    
    }
    rclcpp::shutdown();
    return 0;
}
