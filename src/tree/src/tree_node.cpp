#include "tree/tree_node.hpp"

#include "yaml-cpp/yaml.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include <cmath>
#include <iostream>
#include <unit/unit.hpp>


TreeNode::TreeNode(const rclcpp::NodeOptions &options)
: Node("tree_node", options) {
    // 初始化tf广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 加载平移量
    config_path =
        ament_index_cpp::get_package_share_directory("tree")
        + "/config/offset.yml";
    yaw2pitch = loadTvec("yaw2pitch");
    pitch2fire = loadTvec("pitch2fire");
    fire2camera = loadTvec("fire2camera");

    // 加载欧拉角偏置
    euler_offset = loadEuler("euler_offset");

    // 加载时间戳偏移量
    YAML::Node node = YAML::LoadFile(config_path);
    stamp_offset_ns = node["stamp_offset_ms"].as<int>() * 1000000;

    can_sub_ = this->create_subscription<api::msg::Stm32>(
        "/stm32", rclcpp::SensorDataQoS(),
        [this](const api::msg::Stm32::SharedPtr msg) {
            this->tfCallback(msg);
        }
    );
}

void TreeNode::tfCallback(const api::msg::Stm32::SharedPtr msg) {
    rclcpp::Time stamp = rclcpp::Time(msg->header.stamp) + rclcpp::Duration(0, stamp_offset_ns);
    this->setTF(
        "world", "imu",
        cv::Point3f(0, 0, 0), radian(degree(msg->roll)),
        radian(degree(msg->pitch)), radian(degree(msg->yaw)), stamp
    );
    this->setTF(
        "world", "yaw",
        cv::Point3f(0, 0, 0), radian(degree(msg->roll)),
        radian(0), radian(degree(msg->yaw)), stamp
    );
    this->setTF(
        "yaw", "pitch",
        yaw2pitch, radian(0),
        radian(degree(msg->pitch)), radian(0), stamp
    );
    this->setTF(
        "pitch", "fire",
        pitch2fire, radian(0),
        radian(0), radian(0), stamp
    );
    this->setTF(
        "fire", "camera_imu",
        fire2camera, radian(0),
        radian(0), radian(0), stamp
    );
    this->setTF(
        "camera_imu", "camera",
        cv::Point3f(0, 0, 0), radian(euler_offset[2]),
        radian(euler_offset[1]), radian(euler_offset[0]), stamp
    );
}

cv::Point3f TreeNode::loadTvec(const std::string name) {
    YAML::Node node = YAML::LoadFile(this->config_path);
    return cv::Point3f(
        node[name].as<std::vector<float>>().at(0),
        node[name].as<std::vector<float>>().at(1),
        node[name].as<std::vector<float>>().at(2)
    );
}

std::array<degree, 3> TreeNode::loadEuler(const std::string name) {
    YAML::Node node = YAML::LoadFile(this->config_path);
    auto angles = node[name].as<std::vector<float>>();
    // 打乱顺序以适配相机坐标系
    return {degree(angles[0]), degree(angles[1]), degree(angles[2])};
}

void TreeNode::setTF(
    std::string my_id,
    std::string child_id,
    cv::Point3f tvec,
    radian roll,
    radian pitch,
    radian yaw,
    rclcpp::Time stamp
) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = stamp;
    t.header.frame_id = my_id;
    t.child_frame_id = child_id;

    t.transform.translation.x = tvec.x;
    t.transform.translation.y = tvec.y;
    t.transform.translation.z = tvec.z;

    tf2::Quaternion q;
    q.setRPY(roll.value, pitch.value, yaw.value);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TreeNode)
