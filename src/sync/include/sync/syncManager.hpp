#pragma once

#include <memory>
#include <string>
#include <vector>

#include "sync/coreAlgoBase.hpp"
#include "sync/interpolator.hpp"

// ---------------------------------------------------------------------------
// 估计结果
// ---------------------------------------------------------------------------
struct SyncResult {
    double tau_ms        = 0.0;  ///< 估计的时延（ms），正值表示相机滞后于 IMU
    double peak_value    = 0.0;  ///< 互相关峰值（有符号）；频域方法置 0
    double peak_ratio    = 0.0;  ///< |峰值| / 旁瓣均方根（时域）；频域方法 = fit_r2
    int    num_samples   = 0;    ///< 公共重采样网格上的采样点数
    double fit_r2        = 0.0;  ///< 相位拟合优度 R²（频域用，时域留 0）
    int    num_freq_used = 0;    ///< 参与相位拟合的频点数（频域用，时域留 0）
    bool   valid         = false;
    std::string message;         ///< 可读的状态或错误描述
};

// ---------------------------------------------------------------------------
// SyncManager: 通用前处理（重叠段 -> 均匀网格 -> 插值），
//              算法核心由注入的 CoreAlgoBase 子类实现。
// ---------------------------------------------------------------------------
class SyncManager {
public:
    /// @param algo    算法核心（TimeDomainCalc / FreqDomainCalc），所有权转移
    /// @param method  插值方法
    SyncManager(std::unique_ptr<CoreAlgoBase> algo,
                InterpMethod method = InterpMethod::LINEAR);

    SyncResult estimate(
        const std::vector<double>& t_x,
        const std::vector<double>& x_val,
        const std::vector<double>& t_y,
        const std::vector<double>& y_val,
        double resample_hz   = 100.0,
        double max_offset_ms = 100.0,
        bool   detrend       = true,
        int    min_samples   = 100
    );

private:
    std::unique_ptr<CoreAlgoBase> algo_;
    std::unique_ptr<InterpBase>   interp_;
};
