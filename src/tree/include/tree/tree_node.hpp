#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "api/msg/stm32.hpp"
#include "unit/unit.hpp"

using namespace unit::literals;


class TreeNode : public rclcpp::Node {

public:
    TreeNode(const rclcpp::NodeOptions &options);
    ~TreeNode() = default;

private:
    /**
     * @brief 封装一个函数设置并发布tf变换
     * 
     * @param my_id 当前帧id
     * @param child_id 子帧id
     * @param tvec 到子帧的平移量
     * @param roll 到子帧的roll
     * @param pitch 到子帧的pitch
     * @param yaw 到子帧的yaw
     *
     * @note 两种旋转的理解方式：
     * @note 1. 以 Y P R 的顺序依次旋转, 旋转轴跟着变化
     * @note 1. 以 R P Y 的顺序依次旋转, 旋转轴固定不变
     */
    void setTF(
        std::string my_id,
        std::string child_id,
        cv::Point3f tvec,
        radian roll,
        radian pitch,
        radian yaw,
        rclcpp::Time stamp
    );

    /**
     * @brief 从YAML文件里读取参数以供setTF用
     * 
     * @param name 平移量名
     * @return cv::Point3f 平移量
     */
    cv::Point3f loadTvec(const std::string name);

    /**
     * @brief 从YAML文件里读取欧拉角偏置
     * 
     * @param name 欧拉角名称
     * @return std::array<degree, 3> 欧拉角[yaw, pitch, roll]
     */
    std::array<degree, 3> loadEuler(const std::string name);

    void tfCallback(const api::msg::Stm32::SharedPtr msg);

    std::string config_path;
    cv::Point3f yaw2pitch;
    cv::Point3f pitch2fire;
    cv::Point3f fire2camera;
    std::array<degree, 3> euler_offset{degree(0), degree(0), degree(0)};
    int stamp_offset_ns{0};

    rclcpp::Subscription<api::msg::Stm32>::SharedPtr can_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};
