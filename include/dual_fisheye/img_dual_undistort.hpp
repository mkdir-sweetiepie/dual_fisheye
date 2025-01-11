#ifndef IMG_DUAL_UNDISTORT_HPP
#define IMG_DUAL_UNDISTORT_HPP

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImgDualUndistort : public rclcpp::Node {
 public:
  ImgDualUndistort();

 private:
  // 왼쪽 카메라
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_left;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_left;
  std::string left_img_topic, left_calibrated_topic;
  std::vector<double> left_camera_matrix, left_dist_coeffs;
  cv::Mat left_camera_matrix_mat, left_dist_coeffs_mat, left_new_camera_matrix_mat;
  cv::Mat left_map1, left_map2;

  // 오른쪽 카메라
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_right;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_right;
  std::string right_img_topic, right_calibrated_topic;
  std::vector<double> right_camera_matrix, right_dist_coeffs;
  cv::Mat right_camera_matrix_mat, right_dist_coeffs_mat, right_new_camera_matrix_mat;
  cv::Mat right_map1, right_map2;

  void left_img_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void right_img_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void get_params();

  int img_width, img_height;
  int camera_matrix_rows, camera_matrix_cols;
  int dist_coeffs_rows, dist_coeffs_cols;
};
#endif