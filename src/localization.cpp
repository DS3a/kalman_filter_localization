#include <cstdio>
#include "kalman_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

using std::placeholders::_1;

rclcpp::Node::SharedPtr localization_node = nullptr;

void topic2_callback(const std_msgs::msg::String::SharedPtr msg) {
  std::cout << "received data " << msg->data << "\n\n";
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  localization_node = rclcpp::Node::make_shared("localization");
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr odom_publisher =
    localization_node->create_publisher<std_msgs::msg::String>("topic", 10);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription =
    localization_node->create_subscription<std_msgs::msg::String>("topic2",
                                                                 10,
                                                                 topic2_callback);

  KalmanFilter::LinearKalmanFilter lkf(0.01);
  printf("current position: %f\n\n", lkf.get_position());
  std_msgs::msg::String msg;
  msg.data = "hello";

  lkf.predict(0.01, 0.2, 0.01);

  lkf.measure_position(0.01, 0.51, 0.01);
  rclcpp::spin(localization_node);
  return 0;
}
