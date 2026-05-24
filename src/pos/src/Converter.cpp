#include "pos/Converter.hpp"

#include "yaml-cpp/yaml.h"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include <iostream>


Converter::Converter(const std::string cam_param) {
    YAML::Node node = YAML::LoadFile(cam_param);

    // 解析内参矩阵 (3x3)
    intrinsic_mat = cv::Mat_<double>(3, 3);
    for (int i = 0; i < 3; i++) {
        auto row = node["intrinsic"][i].as<std::vector<double>>();
        std::copy(row.begin(), row.end(), intrinsic_mat.ptr<double>(i));
    }

    // 解析畸变参数
    distortion_vec = node["distortion"].as<std::vector<double>>();

    // 创建 tf2 buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
        std::make_shared<rclcpp::Clock>()
    );

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
}

void Converter::updateStamp(rclcpp::Time stamp) {
    this->img_stamp_ = stamp;
}

cv::Mat Converter::getIntrinsic() {
    return intrinsic_mat;
}

std::vector<double> Converter::getDistortion() {
    return distortion_vec;
}

cv::Point3d Converter::camera2world(cv::Point3d &camera_xyz) const {
    try {
        // 首先尝试使用指定时间戳，添加50ms的时间容忍度
        auto transform = tf_buffer_->lookupTransform(
            "world", "camera",
            this->img_stamp_,
            tf2::durationFromSec(0.005)  // 5ms容忍度
        );

        // 创建camera坐标系下的点
        geometry_msgs::msg::PointStamped point_in;
        point_in.header.frame_id = "camera";
        point_in.point.x = camera_xyz.x;
        point_in.point.y = camera_xyz.y;
        point_in.point.z = camera_xyz.z;

        // 进行坐标变换
        geometry_msgs::msg::PointStamped point_out;
        tf2::doTransform(point_in, point_out, transform);

        // 缓存当前帧的欧拉角
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        tf2::Matrix3x3(q).getEulerYPR(last_yaw_, last_pitch_, last_roll_);

        // 返回world坐标系下的点
        return cv::Point3d(point_out.point.x, point_out.point.y, point_out.point.z);

    } catch (const tf2::TransformException &ex) {
        // 如果指定时间戳失败，尝试使用最新可用时间戳
        try {
            RCLCPP_WARN(rclcpp::get_logger("Converter"), "指定时间戳TF变换失败，尝试使用最新时间戳: %s", ex.what());
            auto transform = tf_buffer_->lookupTransform(
                "world", "camera",
                tf2::TimePointZero  // 使用最新可用时间戳
            );

            geometry_msgs::msg::PointStamped point_in;
            point_in.header.frame_id = "camera";
            point_in.point.x = camera_xyz.x;
            point_in.point.y = camera_xyz.y;
            point_in.point.z = camera_xyz.z;

            geometry_msgs::msg::PointStamped point_out;
            tf2::doTransform(point_in, point_out, transform);

            // 缓存当前帧的欧拉角
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            tf2::Matrix3x3(q).getEulerYPR(last_yaw_, last_pitch_, last_roll_);

            return cv::Point3d(point_out.point.x, point_out.point.y, point_out.point.z);

        } catch (const tf2::TransformException &ex2) {
            RCLCPP_ERROR(rclcpp::get_logger("Converter"), "TF变换失败（回退到最新时间戳也失败）: %s", ex2.what());
            return cv::Point3d(0, 0, 0);
        }
    }
}

void Converter::getLastEulerAngles(double& yaw, double& pitch, double& roll) const {
    yaw = last_yaw_;
    pitch = last_pitch_;
    roll = last_roll_;
}

double Converter::getLastYaw() const {
    return last_yaw_;
}
