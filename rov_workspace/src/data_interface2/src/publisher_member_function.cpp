
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
std::vector<float> odom_sub_vec(13,0);
std::vector<float> odom_pub_vec(13,0);
//std::vector<float> ref_pub_vec(3,0);
//bool record_data = false;
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom3", 10);
    //publisher_1 = this->create_publisher<geometry_msgs::msg::Vector3>("/ref", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
		    "/bluerov2_pid/bluerov2/observer/nlo/odom_ned", 10, std::bind(&MinimalPublisher::odom_callback, this, std::placeholders::_1));
  }

private:
  void timer_callback()
  {
	//Take data from comm, and publish to local ROS network
	//skriv kode her for aa sende via comm
    auto message = nav_msgs::msg::Odometry();
    message.pose.pose.position.x = odom_pub_vec.at(0);
    message.pose.pose.position.y = odom_pub_vec.at(1);
    message.pose.pose.position.z = odom_pub_vec.at(2);
    message.pose.pose.orientation.w = odom_pub_vec.at(3);
    message.pose.pose.orientation.x = odom_pub_vec.at(4);
    message.pose.pose.orientation.y = odom_pub_vec.at(5);
    message.pose.pose.orientation.z = odom_pub_vec.at(6);
    message.twist.twist.linear.x = odom_pub_vec.at(7);
    message.twist.twist.linear.y = odom_pub_vec.at(8);
    message.twist.twist.linear.z = odom_pub_vec.at(9);
    message.twist.twist.angular.x = odom_pub_vec.at(10);
    message.twist.twist.angular.y = odom_pub_vec.at(11);
    message.twist.twist.angular.z = odom_pub_vec.at(12);

    //auto ref_msg = geometry_msgs::msg::Vector3();
    //ref_msg.x = ref_pub_vec.at(0);
    //ref_msg.y = ref_pub_vec.at(1);
    //ref_msg.z = ref_pub_vec.at(2);

    //auto rec_msg = std_msgs::msg::Bool();
    //rec_msg.data = record_data;
    publisher_->publish(message);

    //publisher_1->publish(ref_msg);
    //record_pub->publish(rec_msg);

    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  //rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_1;
  //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr record_pub;
  size_t count_;
  void odom_callback(const nav_msgs::msg::Odometry & msg) const{
	  // Take odometry from local ROS network, and send over comm
	   odom_sub_vec = {msg.pose.pose.position.x,
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
	   //Skriv kode her for aa sende via comm
  }
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
