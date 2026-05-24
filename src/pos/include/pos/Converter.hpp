#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>
#include "eigen3/Eigen/Eigen"
#include <opencv2/core/eigen.hpp>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "unit/unit.hpp"

using namespace unit::literals;


class Converter {

public:
    Converter() = default;
    Converter(const std::string cam_param);
    ~Converter() = default;

    void updateStamp(rclcpp::Time stamp);
    
    cv::Mat getIntrinsic();

    std::vector<double> getDistortion();

    cv::Point3d camera2world(cv::Point3d &camera_xyz) const;

    // 重投影, 相机到图像
    cv::Point2f reproject(cv::Point3d xyz) const;

    void getLastEulerAngles(double& yaw, double& pitch, double& roll) const;
    double getLastYaw() const;

private:
    cv::Mat_<double> intrinsic_mat;
    std::vector<double> distortion_vec;

    rclcpp::Time img_stamp_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    mutable double last_yaw_ = 0.0;
    mutable double last_pitch_ = 0.0;
    mutable double last_roll_ = 0.0;

};
