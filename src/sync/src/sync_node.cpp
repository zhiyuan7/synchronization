#include "sync/sync_node.hpp"
#include "sync/interpolator.hpp"

#include <chrono>
#include <memory>
#include <string>

// ---------------------------------------------------------------------------
// 构造函数
// ---------------------------------------------------------------------------

SyncNode::SyncNode(const rclcpp::NodeOptions & options)
: Node("sync_node", options)
{
    // ---- 参数 ----
    collect_seconds_ = declare_parameter<double>("collect_seconds", 5.0);
    resample_hz_     = declare_parameter<double>("resample_hz", 100.0);
    max_offset_ms_   = declare_parameter<double>("max_offset_ms", 100.0);
    detrend_         = declare_parameter<bool>("detrend", true);
    min_samples_     = declare_parameter<int>("min_samples", 200);
    std::string interp_str = declare_parameter<std::string>("interp_method", "linear");
    InterpMethod interp_method = interpMethodFromString(interp_str);

    // ---- 算法实例（时域计算）----
    calc_ = std::make_unique<时域估计>(interp_method);

    // ---- 订阅者 ----
    // /pos：已由 pos_node 完成 PnP 求解，camera.y 为水平坐标
    pos_sub_ = create_subscription<api::msg::PoseResults>(
        "/pos", 10,
        [this](api::msg::PoseResults::SharedPtr msg) { onPos(msg); }
    );
    stm32_sub_ = create_subscription<api::msg::Stm32>(
        "/stm32", rclcpp::SensorDataQoS(),
        [this](api::msg::Stm32::SharedPtr msg) { onStm32(msg); }
    );

    // ---- 周期定时器：每隔 collect_seconds_ 秒触发一次估计 ----
    trigger_timer_ = create_wall_timer(
        std::chrono::duration<double>(collect_seconds_),
        [this]() {
            if (!estimated_) runEstimation();
        }
    );

    RCLCPP_INFO(get_logger(),
        "[SyncNode] Ready. Collecting %.0f s of data "
        "(resample=%.0f Hz, search=±%.0f ms, interp=%s)...",
        collect_seconds_, resample_hz_, max_offset_ms_, interpMethodName(interp_method));
}

// ---------------------------------------------------------------------------
// /pos 话题回调（api::msg::PoseResults，由 pos_node 发布）
// ---------------------------------------------------------------------------

void SyncNode::onPos(api::msg::PoseResults::SharedPtr msg)
{
    if (estimated_ || msg->pose_results.empty()) return;

    // ---- 将 ROS 消息转换为内部类型 CouplePos ----
    CouplePos positions;
    for (const auto& pose : msg->pose_results) {
        SinglePos sp;
        sp.camera = cv::Point3d(pose.camera.x, pose.camera.y, pose.camera.z);
        sp.world  = cv::Point3d(pose.world.x,  pose.world.y,  pose.world.z);
        sp.yaw    = pose.yaw;
        sp.id     = pose.id;
        sp.pnp_reproj_err = pose.pnp_reproj_err;
        positions.push_back(sp);
    }

    // ---- 选取相机系范数最小（最近）的目标 ----
    SinglePos nearest = utils::seek::minElementByValue(positions,
        [](const SinglePos& p) {
            return p.camera.x * p.camera.x
                 + p.camera.y * p.camera.y
                 + p.camera.z * p.camera.z;
        }
    );

    double t_sec = msg->header.stamp.sec
                 + msg->header.stamp.nanosec * 1e-9;
    if (t_sec <= 0.0) return;
    if (!t_cam_.empty() && t_sec <= t_cam_.back()) return;

    t_cam_.push_back(t_sec);
    x_cam_.push_back(nearest.camera.y);  // 相机系水平坐标 [mm]，用作相关信号
}

// ---------------------------------------------------------------------------
// /stm32 话题回调
// ---------------------------------------------------------------------------

void SyncNode::onStm32(api::msg::Stm32::SharedPtr msg)
{
    if (estimated_) return;

    double t_sec = msg->header.stamp.sec
                 + msg->header.stamp.nanosec * 1e-9;
    if (t_sec <= 0.0) return;
    if (!t_imu_.empty() && t_sec <= t_imu_.back()) return;

    t_imu_.push_back(t_sec);
    y_imu_.push_back(static_cast<double>(msg->yaw));  // degrees
}

// ---------------------------------------------------------------------------
// 核心估计
// ---------------------------------------------------------------------------

void SyncNode::runEstimation()
{
    estimated_ = true;

    RCLCPP_INFO(get_logger(),
        "[SyncNode] Trigger fired. Camera samples: %zu  IMU samples: %zu",
        t_cam_.size(), t_imu_.size());

    auto res = calc_->estimate(
        t_cam_, x_cam_, t_imu_, y_imu_,
        resample_hz_, max_offset_ms_, detrend_, min_samples_
    );

    // ---- 打印结果 ----
    RCLCPP_INFO(get_logger(), "============================================================");
    if (!res.valid) {
        RCLCPP_ERROR(get_logger(), "[SyncNode] Estimation failed: %s", res.message.c_str());
        RCLCPP_INFO(get_logger(), "============================================================");
        return;
    }

    long long suggested_ms = static_cast<long long>(std::round(res.tau_ms));

    RCLCPP_INFO(get_logger(), "[SyncNode] === TDE Result ===");
    RCLCPP_INFO(get_logger(), "  Estimated tau:        %.2f ms", res.tau_ms);
    RCLCPP_INFO(get_logger(), "  Peak value:           %.4f  (sign: %s)",
        res.peak_value, res.peak_value >= 0 ? "positive" : "negative");
    RCLCPP_INFO(get_logger(), "  Peak sharpness ratio: %.2f  (>3 is reliable)",
        res.peak_ratio);
    RCLCPP_INFO(get_logger(), "  Samples on grid:      %d", res.num_samples);
    RCLCPP_INFO(get_logger(), "  Camera samples raw:   %zu", t_cam_.size());
    RCLCPP_INFO(get_logger(), "  IMU samples raw:      %zu", t_imu_.size());
    RCLCPP_INFO(get_logger(), "--");
    RCLCPP_INFO(get_logger(),
        "  >>> Suggested  stamp_offset_ms: %lld  (write to offset.yml) <<<",
        suggested_ms);
    RCLCPP_INFO(get_logger(),
        "  Note: positive = camera lags IMU (typical USB camera case).");
    if (res.peak_ratio < 3.0) {
        RCLCPP_WARN(get_logger(),
            "  [!] Low sharpness (%.2f < 3). Ensure sufficient gimbal motion "
            "during recording, and check that target stays in frame.", res.peak_ratio);
    }
    RCLCPP_INFO(get_logger(), "============================================================");

    // ---- 清空缓冲区，准备下一轮收集 ----
    t_cam_.clear(); x_cam_.clear();
    t_imu_.clear(); y_imu_.clear();
    estimated_ = false;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SyncNode)
