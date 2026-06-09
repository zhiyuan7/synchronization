#include "sync/syncManager.hpp"
#include "sync/interpolator.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <memory>

// ---------------------------------------------------------------------------
// SyncManager 构造函数
// ---------------------------------------------------------------------------

SyncManager::SyncManager(std::unique_ptr<CoreAlgoBase> algo, InterpMethod method)
    : algo_(std::move(algo)), interp_(makeInterpolator(method))
{}

// ---------------------------------------------------------------------------
// SyncManager::estimate
// ---------------------------------------------------------------------------

SyncResult SyncManager::estimate(
    const std::vector<double>& t_x,
    const std::vector<double>& x_val,
    const std::vector<double>& t_y,
    const std::vector<double>& y_val,
    double resample_hz,
    double max_offset_ms,
    bool   detrend,
    int    min_samples)
{
    SyncResult res{};
    res.valid = false;

    // 输入基本有效性检查（防止后续算法崩溃）
    if (t_x.size() < 2 || t_y.size() < 2 ||
        t_x.size() != x_val.size() || t_y.size() != y_val.size()) {
        res.message = "insufficient or mismatched input data";
        return res;
    }

    // --- 1. 计算重叠时间段 ---
    double t_start = std::max(t_x.front(), t_y.front());
    double t_end   = std::min(t_x.back(),  t_y.back());
    if (t_end <= t_start + 1e-6) {
        res.message = "no time overlap between camera and IMU signals";
        return res;
    }

    // --- 2. 以 resample_hz 构建均匀时间网格 ---
    double Ts = 1.0 / resample_hz;
    int N = static_cast<int>((t_end - t_start) / Ts) + 1;
    if (N < min_samples) {
        std::ostringstream oss;
        oss << "only " << N << " grid samples in overlap (need " << min_samples << ")";
        res.message = oss.str();
        return res;
    }

    std::vector<double> t_grid(N);
    for (int i = 0; i < N; ++i) t_grid[i] = t_start + i * Ts;

    // --- 3. 插值到公共网格 ---
    std::vector<double> xn = interp_->interp(t_x, x_val, t_grid);
    std::vector<double> yn = interp_->interp(t_y, y_val, t_grid);

    // --- 4. 委托注入的算法核心执行实际计算 ---
    CoreResult core = algo_->compute(xn, yn, N, resample_hz, max_offset_ms, detrend);
    if (!core.valid) {
        res.message = core.message;
        return res;
    }

    // --- 5. 计算时延 ---
    // 推导：x[n] = y[n−k₀]  →  R_xy 在 m̂ = −k₀ 处取最大值
    //       τ̂ = −m̂·Ts（正值表示相机滞后于 IMU）
    double tau_ms = -core.tau * 1000.0;

    res.tau_ms        = tau_ms;
    res.peak_value    = core.peak_value;
    res.peak_ratio    = core.peak_ratio;
    res.num_samples   = N;
    res.fit_r2        = core.fit_r2;
    res.num_freq_used = core.num_freq_used;
    res.valid         = true;
    res.message       = core.message;
    return res;
}
