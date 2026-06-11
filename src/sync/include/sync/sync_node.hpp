#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "api/msg/pose_results.hpp"
#include "api/msg/stm32.hpp"
#include "unit/types.hpp"
#include "utils/utils.hpp"

#include "sync/syncManager.hpp"

class SyncNode : public rclcpp::Node {
public:
    explicit SyncNode(const rclcpp::NodeOptions & options);
    ~SyncNode() = default;

private:
    // --- ROS 回调函数 ---
    void onPos(api::msg::PoseResults::SharedPtr msg);
    void onStm32(api::msg::Stm32::SharedPtr msg);

    // --- 核心估计，定时器触发后调用一次 ---
    void runEstimation();

    // --- 订阅者与定时器 ---
    rclcpp::Subscription<api::msg::PoseResults>::SharedPtr pos_sub_;
    rclcpp::Subscription<api::msg::Stm32>::SharedPtr        stm32_sub_;
    rclcpp::TimerBase::SharedPtr                             trigger_timer_;

    // --- 信号缓冲区 ---
    std::vector<double> t_cam_;   ///< 相机检测时间戳 [s]
    std::vector<double> x_cam_;   ///< 相机信号：PnP 求解的 camera.y [mm]
    std::vector<double> t_imu_;   ///< IMU 时间戳 [s]
    std::vector<double> y_imu_;   ///< IMU 信号：stm32 偏航角 [deg]

    // --- 参数 ---
    double collect_seconds_;
    double resample_hz_;
    double coarse_resample_hz_;
    double fine_resample_hz_;
    double max_offset_ms_;
    bool   detrend_;
    int    min_samples_;

    // --- 算法实现（SyncManager 注入具体的 CoreAlgoBase）---
    std::unique_ptr<SyncManager> calc_;

    bool estimated_ = false;  ///< 首次估计完成后防止重复执行
};
