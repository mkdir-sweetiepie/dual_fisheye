#include "../include/dual_fisheye/img_dual_undistort.hpp"

ImgDualUndistort::ImgDualUndistort() : Node("img_dual_undistort") {
  get_params();

  sub_img_left = this->create_subscription<sensor_msgs::msg::Image>(left_img_topic.c_str(), 1, std::bind(&ImgDualUndistort::left_img_callback, this, std::placeholders::_1));
  pub_img_left = this->create_publisher<sensor_msgs::msg::Image>(left_calibrated_topic.c_str(), 10);

  sub_img_right = this->create_subscription<sensor_msgs::msg::Image>(right_img_topic.c_str(), 1, std::bind(&ImgDualUndistort::right_img_callback, this, std::placeholders::_1));
  pub_img_right = this->create_publisher<sensor_msgs::msg::Image>(right_calibrated_topic.c_str(), 10);

  RCLCPP_INFO(this->get_logger(), "ImgDualUndistort node has been created");
}

void ImgDualUndistort::left_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    cv::Mat undistorted;
    cv::remap(img, undistorted, left_map1, left_map2, cv::INTER_LINEAR);

    auto img_undistorted_msg = cv_bridge::CvImage(msg->header, "bgr8", undistorted).toImageMsg();
    pub_img_left->publish(*img_undistorted_msg);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
}

void ImgDualUndistort::right_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    cv::Mat undistorted;
    cv::remap(img, undistorted, right_map1, right_map2, cv::INTER_LINEAR);

    auto img_undistorted_msg = cv_bridge::CvImage(msg->header, "bgr8", undistorted).toImageMsg();
    pub_img_right->publish(*img_undistorted_msg);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
}

void ImgDualUndistort::get_params() {
  // 이미지 크기 파라미터
  img_width = this->declare_parameter<int>("img_width", 736);
  img_height = this->declare_parameter<int>("img_height", 736);

  // 토픽 이름 파라미터
  left_img_topic = this->declare_parameter<std::string>("left_img_topic", "/camera1/left/camera/image_raw");
  left_calibrated_topic = this->declare_parameter<std::string>("left_calibrated_topic", "/left/image_calibrated");
  right_img_topic = this->declare_parameter<std::string>("right_img_topic", "/camera1/right/camera/image_raw");
  right_calibrated_topic = this->declare_parameter<std::string>("right_calibrated_topic", "/right/image_calibrated");

  // 행렬 크기 파라미터
  camera_matrix_rows = this->declare_parameter<int>("camera_matrix_rows", 3);
  camera_matrix_cols = this->declare_parameter<int>("camera_matrix_cols", 3);
  dist_coeffs_rows = this->declare_parameter<int>("dist_coeffs_rows", 1);
  dist_coeffs_cols = this->declare_parameter<int>("dist_coeffs_cols", 4);

  // 왼쪽 카메라 캘리브레이션 파라미터
  left_camera_matrix = this->declare_parameter<std::vector<double>>("left_camera_matrix", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  left_dist_coeffs = this->declare_parameter<std::vector<double>>("left_dist_coeffs", {0.0, 0.0, 0.0, 0.0});

  // 오른쪽 카메라 캘리브레이션 파라미터
  right_camera_matrix = this->declare_parameter<std::vector<double>>("right_camera_matrix", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  right_dist_coeffs = this->declare_parameter<std::vector<double>>("right_dist_coeffs", {0.0, 0.0, 0.0, 0.0});

  // OpenCV 행렬 변환
  left_camera_matrix_mat = cv::Mat(camera_matrix_rows, camera_matrix_cols, CV_64F, left_camera_matrix.data());
  left_dist_coeffs_mat = cv::Mat(dist_coeffs_rows, dist_coeffs_cols, CV_64F, left_dist_coeffs.data());
  right_camera_matrix_mat = cv::Mat(camera_matrix_rows, camera_matrix_cols, CV_64F, right_camera_matrix.data());
  right_dist_coeffs_mat = cv::Mat(dist_coeffs_rows, dist_coeffs_cols, CV_64F, right_dist_coeffs.data());

  cv::Size image_size(img_width, img_height);

  // undistort map 생성
  cv::fisheye::initUndistortRectifyMap(left_camera_matrix_mat, left_dist_coeffs_mat, cv::Mat::eye(3, 3, CV_64F), left_camera_matrix_mat, cv::Size(img_width, img_height), CV_16SC2, left_map1, left_map2);
  cv::fisheye::initUndistortRectifyMap(right_camera_matrix_mat, right_dist_coeffs_mat, cv::Mat::eye(3, 3, CV_64F), right_camera_matrix_mat, cv::Size(img_width, img_height), CV_16SC2, right_map1, right_map2);

  RCLCPP_INFO(this->get_logger(), "Left Image Topic: %s", left_img_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Right Image Topic: %s", right_img_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Image Width: %d", img_width);
  RCLCPP_INFO(this->get_logger(), "Image Height: %d", img_height);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImgDualUndistort>());
  rclcpp::shutdown();
  return 0;
}