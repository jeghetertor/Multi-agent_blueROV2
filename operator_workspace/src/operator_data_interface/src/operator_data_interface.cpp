#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
std::vector<float> ref_vec(3,0);
std::vector<float> odom2_vec(13,0);
std::vector<float> odom3_vec(13,0);
bool record_data = false;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("operator_data_interface"), count_(0)
    {
      odom2_pub = this->create_publisher<nav_msgs::msg::Odometry>("/bluerov2_pid/bluerov2/observer/nlo/odom_ned", 10);
      odom3_pub = this->create_publisher<nav_msgs::msg::Odometry>("/bluerov3_pid/bluerov2/observer/nlo/odom_ned", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));

      ref_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
		    "/ref", 10, std::bind(&MinimalPublisher::ref_callback, this, std::placeholders::_1));

      odom3_sub = this->create_subscription<nav_msgs::msg::Odometry>(
		    "/bluerov2_pid/bluerov3/observer/nlo/odom_ned", 10, std::bind(&MinimalPublisher::odom3_callback, this, std::placeholders::_1));
      odom2_sub = this->create_subscription<nav_msgs::msg::Odometry>(
		    "/bluerov2_pid/bluerov2/observer/nlo/odom_ned", 10, std::bind(&MinimalPublisher::odom2_callback, this, std::placeholders::_1));
      record_sub = this->create_subscription<std_msgs::msg::Bool>(
		    "/record_data", 10, std::bind(&MinimalPublisher::rec_callback, this, std::placeholders::_1));

    }

  private:
    void timer_callback()
    {
      auto odom2_msg = nav_msgs::msg::Odometry();
      auto odom3_msg = nav_msgs::msg::Odometry();
	// Insert kode her for aa hente odom2 og odom3 vec fra comm
      odom2_msg.pose.pose.position.x = odom2_vec.at(0);
      odom2_msg.pose.pose.position.y = odom2_vec.at(1);
      odom2_msg.pose.pose.position.z = odom2_vec.at(2);
      odom2_msg.pose.pose.orientation.w = odom2_vec.at(3);
      odom2_msg.pose.pose.orientation.x = odom2_vec.at(4);
      odom2_msg.pose.pose.orientation.y = odom2_vec.at(5);
      odom2_msg.pose.pose.orientation.z = odom2_vec.at(6);
      odom2_msg.twist.twist.linear.x = odom2_vec.at(7);
      odom2_msg.twist.twist.linear.y = odom2_vec.at(8);
      odom2_msg.twist.twist.linear.z = odom2_vec.at(9);
      odom2_msg.twist.twist.angular.x = odom2_vec.at(10);
      odom2_msg.twist.twist.angular.y =odom2_vec.at(11);
      odom2_msg.twist.twist.angular.z = odom2_vec.at(12);
  
      odom3_msg.pose.pose.position.x = odom3_vec.at(0);
      odom3_msg.pose.pose.position.y = odom3_vec.at(1);
      odom3_msg.pose.pose.position.z = odom3_vec.at(2);
      odom3_msg.pose.pose.orientation.w = odom3_vec.at(3);
      odom3_msg.pose.pose.orientation.x = odom3_vec.at(4);
      odom3_msg.pose.pose.orientation.y = odom3_vec.at(5);
      odom3_msg.pose.pose.orientation.z = odom3_vec.at(6);
      odom3_msg.twist.twist.linear.x = odom3_vec.at(7);
      odom3_msg.twist.twist.linear.y = odom3_vec.at(8);
      odom3_msg.twist.twist.linear.z = odom3_vec.at(9);
      odom3_msg.twist.twist.angular.x = odom3_vec.at(10);
      odom3_msg.twist.twist.angular.y = odom3_vec.at(11);
      odom3_msg.twist.twist.angular.z = odom3_vec.at(12);

      odom2_pub->publish(odom2_msg);
      odom3_pub->publish(odom3_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom2_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom3_pub;
    size_t count_;
    
    void ref_callback(const geometry_msgs::msg::Vector3 & msg) const{
	ref_vec.at(0) = msg.x;
	ref_vec.at(1) = msg.y;
	ref_vec.at(2) = msg.z;
	//Skriv noe for aa sende via comm
    }
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ref_sub;

    void odom3_callback(const nav_msgs::msg::Odometry & msg) const{
	odom3_vec = {msg.pose.pose.position.x,
		  msg.pose.pose.position.y,
		  msg.pose.pose.position.z,
		  msg.pose.pose.orientation.w,
		  msg.pose.pose.orientation.x,
		  msg.pose.pose.orientation.y,
		  msg.pose.pose.orientation.z,
		  msg.twist.twist.linear.x,
		  msg.twist.twist.linear.y,
		  msg.twist.twist.linear.z,
		  msg.twist.twist.angular.x,
		  msg.twist.twist.angular.y,
		  msg.twist.twist.angular.z};    
	// Skriv node for aa sende via comm
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom3_sub;
    void odom2_callback(const nav_msgs::msg::Odometry & msg) const{
	odom2_vec = {msg.pose.pose.position.x,
		  msg.pose.pose.position.y,
		  msg.pose.pose.position.z,
		  msg.pose.pose.orientation.w,
		  msg.pose.pose.orientation.x,
		  msg.pose.pose.orientation.y,
		  msg.pose.pose.orientation.z,
		  msg.twist.twist.linear.x,
		  msg.twist.twist.linear.y,
		  msg.twist.twist.linear.z,
		  msg.twist.twist.angular.x,
		  msg.twist.twist.angular.y,
		  msg.twist.twist.angular.z};    
	// Skriv node for aa sende via comm
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom2_sub;

    void rec_callback(const std_msgs::msg::Bool & msg) const{
    	record_data = msg.data;	
	// Skriv noe for aa sende via comm
    }
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr record_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
