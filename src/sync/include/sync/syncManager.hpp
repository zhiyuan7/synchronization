#pragma once

#include <memory>
#include <string>
#include <vector>

#include "sync/TimeDomainAlgo.hpp"
#include "sync/interpolator.hpp"

// ---------------------------------------------------------------------------
// 估计结果
// ---------------------------------------------------------------------------
struct SyncResult {
    double tau_ms      = 0.0;  ///< 估计的时延（ms），正值表示相机滞后于 IMU
    double peak_value  = 0.0;  ///< 互相关峰值（有符号）
    double peak_ratio  = 0.0;  ///< |峰值| / 旁瓣均方根，衡量峰值锐利度
    int    num_samples = 0;    ///< 公共重采样网格上的采样点数
    bool   valid       = false;
    std::string message;       ///< 可读的状态或错误描述
};

// ---------------------------------------------------------------------------
// 同步算法抽象基类
// ---------------------------------------------------------------------------
class SyncCalcBase {
public:
    virtual ~SyncCalcBase() = default;

    /// 估计两路非均匀采样标量信号之间的时延。
    ///
    /// 符号约定：tau_ms > 0 ↔ 相机滞后于 IMU tau_ms 毫秒
    ///
    /// @param t_x           相机时间戳 [s]，必须升序排列
    /// @param x_val         相机信号值（如 camera.y，单位 mm）
    /// @param t_y           IMU 时间戳 [s]，必须升序排列
    /// @param y_val         IMU 信号值（如 yaw，单位 deg）
    /// @param resample_hz   均匀重采样频率 [Hz]
    /// @param max_offset_ms 时延搜索半宽 [ms]
    /// @param detrend       是否在相关前去除线性趋势
    /// @param min_samples   公共网格上所需的最少采样点数
    virtual SyncResult estimate(
        const std::vector<double>& t_x,
        const std::vector<double>& x_val,
        const std::vector<double>& t_y,
        const std::vector<double>& y_val,
        double resample_hz   = 100.0,
        double max_offset_ms = 100.0,
        bool   detrend       = true,
        int    min_samples   = 100
    ) = 0;
};

// ---------------------------------------------------------------------------
// 时域估计：完整估计流程派生类（步骤 1~3 + 委托 时域计算 执行步骤 4~6）
// ---------------------------------------------------------------------------
class 时域估计 : public SyncCalcBase {
public:
    explicit 时域估计(InterpMethod method = InterpMethod::LINEAR);

    SyncResult estimate(
        const std::vector<double>& t_x,
        const std::vector<double>& x_val,
        const std::vector<double>& t_y,
        const std::vector<double>& y_val,
        double resample_hz   = 100.0,
        double max_offset_ms = 100.0,
        bool   detrend       = true,
        int    min_samples   = 100
    ) override;

private:
    std::unique_ptr<InterpBase> interp_;
};
