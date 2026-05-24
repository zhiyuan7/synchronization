#pragma once

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "PosSolver.hpp"
#include "api/msg/stm32.hpp"
#include "api/msg/detect_results.hpp"
#include "api/msg/pose_results.hpp"


class PosNode : public rclcpp::Node {

public:
    PosNode(const rclcpp::NodeOptions &options);
    ~PosNode() = default;

    void stmCallback(const api::msg::Stm32::SharedPtr msg);

    void detectCallback(const api::msg::DetectResults::SharedPtr msg);

private:
    std::shared_ptr<PosSolver> pos_solver;

    rclcpp::Subscription<api::msg::Stm32>::SharedPtr yaw_sub_;
    rclcpp::Subscription<api::msg::DetectResults>::SharedPtr detect_sub_;
    rclcpp::Publisher<api::msg::PoseResults>::SharedPtr pos_pub_;

};
