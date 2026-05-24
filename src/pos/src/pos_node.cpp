#include "pos/pos_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <rclcpp/logging.hpp>
#include <unit/types.hpp>
#include <unit/unit.hpp>

PosNode::PosNode(const rclcpp::NodeOptions &options)
: Node("pos_node", options) {
    // 初始化坐标解算器 (加载相机参数罢)
    pos_solver = std::make_shared<PosSolver>(
        ament_index_cpp::get_package_share_directory("pos")
        + "/config/cam_param.yml",
        ament_index_cpp::get_package_share_directory("tree")
        + "/config/offset.yml"
    );

    // 订阅yaw数据
    yaw_sub_ = this->create_subscription<api::msg::Stm32>(
        "/stm32", rclcpp::SensorDataQoS(),
        [this](const api::msg::Stm32::SharedPtr msg) {
            this->stmCallback(msg);
        }
    );
    
    // 订阅检测结果
    detect_sub_ = this->create_subscription<api::msg::DetectResults>(
        "/detect/detections", 1,
        [this](const api::msg::DetectResults::SharedPtr msg) {
            this->detectCallback(msg);
        }
    );
    
    // 创建位置发布器
    pos_pub_ = this->create_publisher<api::msg::PoseResults>("/pos", 1);
}

void PosNode::stmCallback(const api::msg::Stm32::SharedPtr) {
}

void PosNode::detectCallback(const api::msg::DetectResults::SharedPtr msg) {
    pos_solver->updateStmAndStamp(
        rclcpp::Time(msg->header.stamp.sec*1e9 + msg->header.stamp.nanosec)
    );

    CoupleDetections detections;
    for (const auto &detect: msg->detect_results) {
        SingleDetection tmp;
        for (const auto& kpt : detect.kpts) {
            tmp.kpts.emplace_back(kpt.x, kpt.y);
        }
        tmp.is_big = detect.is_big;
        tmp.color = static_cast<TeamColor>(detect.color);
        tmp.id = detect.id;

        detections.push_back(tmp);
    }

    CouplePos res = pos_solver->getPosition(detections);

    // 转换为ROS消息并发布
    api::msg::PoseResults pos_msg;
    pos_msg.header = msg->header;
    
    for (size_t i = 0; i < res.size(); ++i) {
        api::msg::PoseResult single_pos;
        single_pos.id = detections[i].id;
        single_pos.world.x = res[i].world.x;
        single_pos.world.y = res[i].world.y;
        single_pos.world.z = res[i].world.z;
        single_pos.camera.x = res[i].camera.x;
        single_pos.camera.y = res[i].camera.y;
        single_pos.camera.z = res[i].camera.z;
        single_pos.yaw = res[i].yaw;
        single_pos.pnp_reproj_err = res[i].pnp_reproj_err;
        
        pos_msg.pose_results.push_back(single_pos);
    }
    
    pos_pub_->publish(pos_msg);
    
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PosNode)
