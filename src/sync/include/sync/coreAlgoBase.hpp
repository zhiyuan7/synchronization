#pragma once

#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// 核心算法计算结果
// ---------------------------------------------------------------------------
struct CoreResult {
    int    m_hat      = 0;    ///< 互相关峰值对应的延迟采样数 m̂
    double peak_value = 0.0;  ///< 互相关峰值（有符号）
    double peak_ratio = 0.0;  ///< |峰值| / 旁瓣均方根，衡量峰值锐利度
    bool   valid      = false;
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
