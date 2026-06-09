#pragma once

#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// 核心算法计算结果
// ---------------------------------------------------------------------------
struct CoreResult {
    double tau           = 0.0;  ///< 延时估计 [s]，= m_hat/hz（时域）或 slope/hz（频域）；tau_ms = -tau*1000
    double peak_value    = 0.0;  ///< 【时域专用】互相关峰值（有符号）
    double peak_ratio    = 0.0;  ///< 【时域专用】|峰值| / 旁瓣均方根
    double fit_r2        = 0.0;  ///< 【频域专用】相位拟合优度 R²
    int    num_freq_used = 0;    ///< 【频域专用】参与拟合的频点数
    bool   valid         = false;
    std::string message;
};

// ---------------------------------------------------------------------------
// 核心算法抽象基类
// 输入：已插值到公共网格的两路信号 xn / yn
// 职责：预处理 → 相关计算 → 峰值搜索
// ---------------------------------------------------------------------------
class CoreAlgoBase {
public:
    virtual ~CoreAlgoBase() = default;

    /// @param xn            相机信号（均匀网格，长度 N，函数内可被修改）
    /// @param yn            IMU 信号（均匀网格，长度 N，函数内可被修改）
    /// @param N             网格点数
    /// @param resample_hz   重采样频率 [Hz]
    /// @param max_offset_ms 时延搜索半宽 [ms]
    /// @param detrend       是否在相关前去除线性趋势
    virtual CoreResult compute(
        std::vector<double>& xn,
        std::vector<double>& yn,
        int    N,
        double resample_hz,
        double max_offset_ms,
        bool   detrend
    ) = 0;
};
