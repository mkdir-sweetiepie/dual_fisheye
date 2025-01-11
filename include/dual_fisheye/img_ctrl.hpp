#ifndef IMG_CTRL_HPP
#define IMG_CTRL_HPP

#include <cv_bridge/cv_bridge.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgcodecs.hpp>

class ImgCtrl : public rclcpp_lifecycle::LifecycleNode {
 public:
  ImgCtrl(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ImgCtrl();
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &state);

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_img_left;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_img_right;

  void img_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void get_params();

  std::string camera_name, topic, compressed_topic, compressed_depth_topic, camera_info_topic, frame_id, resolution_str, resolution_str2, format;
  double fps;
  std::string left_topic, right_topic;

  int original_width, original_height, split_width, split_height;
};
#endif