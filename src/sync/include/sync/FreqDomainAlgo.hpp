#pragma once

#include "sync/coreAlgoBase.hpp"

// ---------------------------------------------------------------------------
// FreqDomainCalc：基于互功率谱相位斜率拟合的频域核心算法
//
// 方法（文档 §三.4）：
//   S_xy[k] = conj(X[k])·Y[k]
//   对正频段相位做 unwrap 后，最小二乘拟合 φ[k] ≈ a·ω_k + b
//   延时估计：tau_samples = -a（样本），tau_ms = -tau_samples·(1000/hz)
// ---------------------------------------------------------------------------
class FreqDomainCalc : public CoreAlgoBase {
public:
    explicit FreqDomainCalc(bool apply_window = true, double max_fit_hz = 0.0);

    CoreResult compute(
        std::vector<double>& xn,
        std::vector<double>& yn,
        int    N,
        double resample_hz,
        double max_offset_ms,
        bool   detrend
    ) override;

private:
    bool   apply_window_;
    double max_fit_hz_;   ///< 拟合使用的最高物理频率 [Hz]，≤0 表示不限制（使用所有正频点）
};
