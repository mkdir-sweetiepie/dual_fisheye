#include "../include/dual_fisheye/img_ctrl.hpp"

ImgCtrl::ImgCtrl(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("img_ctrl", options) { RCLCPP_INFO(this->get_logger(), "ImgCtrl node has been created"); }

ImgCtrl::~ImgCtrl() { RCLCPP_INFO(this->get_logger(), "ImgCtrl node has been destroyed"); }

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImgCtrl::on_configure(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Configuring");

  get_params();

  sub_img = this->create_subscription<sensor_msgs::msg::Image>(topic.c_str(), 1, std::bind(&ImgCtrl::img_callback, this, std::placeholders::_1));
  pub_img_left = this->create_publisher<sensor_msgs::msg::Image>(left_topic.c_str(), 10);
  pub_img_right = this->create_publisher<sensor_msgs::msg::Image>(right_topic.c_str(), 10);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImgCtrl::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(), "Activating");

  pub_img_left->on_activate();
  pub_img_right->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImgCtrl::on_deactivate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(), "Deactivating");

  pub_img_left->on_deactivate();
  pub_img_right->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImgCtrl::on_cleanup(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up");

  sub_img.reset();
  pub_img_left.reset();
  pub_img_right.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImgCtrl::on_shutdown(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(), "Shutting down");

  return on_cleanup(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImgCtrl::on_error(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(), "Error");

  return on_cleanup(state);
}

void ImgCtrl::img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    if (img.cols != original_width || img.rows != original_height) {
      RCLCPP_INFO(this->get_logger(), "Resizing image from %dx%d to %dx%d", img.cols, img.rows, original_width, original_height);
      return;
    }
    cv::Rect roi_left(0, 0, split_width, split_height);  // x, y, width, height
    cv::Rect roi_right(split_width, 0, split_width, split_height);

    cv::Mat img_left = img(roi_left);
    cv::Mat img_right = img(roi_right);

    auto left_msg = cv_bridge::CvImage(msg->header, "bgr8", img_left).toImageMsg();
    auto right_msg = cv_bridge::CvImage(msg->header, "bgr8", img_right).toImageMsg();

    if (pub_img_left->is_activated()) {
      pub_img_left->publish(*left_msg);
    }
    if (pub_img_right->is_activated()) {
      pub_img_right->publish(*right_msg);
    }
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
}

void ImgCtrl::get_params() {
  std::string topic_str = this->declare_parameter<std::string>("topic", "/camera/image_raw");
  // std::string compressed_topic_str = this->declare_parameter<std::string>("compressed_topic", "/camera/compressed");
  // std::string compressed_depth_topic_str = this->declare_parameter<std::string>("compressed_depth_topic", "/camera/compressedDepth");

  camera_name = this->declare_parameter<std::string>("camera_name", "camera1");
  topic = "/" + camera_name + topic_str;
  // compressed_topic = "/" + camera_name + compressed_topic_str;
  // compressed_depth_topic = "/" + camera_name + compressed_depth_topic_str;
  camera_info_topic = "/" + camera_name + "/info";

  frame_id = this->declare_parameter<std::string>("frame_id", "camera");
  resolution_str = this->declare_parameter<std::string>("resolution", "1472x736");
  resolution_str2 = this->declare_parameter<std::string>("split_resolution", "736x736");
  format = this->declare_parameter<std::string>("format", "MJPEG");
  fps = this->declare_parameter<double>("fps", 30.0);

  left_topic = "/" + camera_name + "/left" + topic_str;
  right_topic = "/" + camera_name + "/right" + topic_str;

  sscanf(resolution_str.c_str(), "%dx%d", &original_width, &original_height);
  sscanf(resolution_str2.c_str(), "%dx%d", &split_width, &split_height);

  RCLCPP_INFO(this->get_logger(), "Original Image Size : %s", resolution_str.c_str());
  RCLCPP_INFO(this->get_logger(), "Splitting to Size : %s", resolution_str2.c_str());
  RCLCPP_INFO(this->get_logger(), "Left Image Topic : %s", left_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Right Image Topic : %s", right_topic.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImgCtrl>();

  auto configure_result = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configure_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure");
    return 1;
  }

  auto activate_result = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activate_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to activate");
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}