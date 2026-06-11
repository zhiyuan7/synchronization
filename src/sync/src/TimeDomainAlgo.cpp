#include "sync/TimeDomainAlgo.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

// ---------------------------------------------------------------------------
// 内部辅助函数（文件作用域）
// ---------------------------------------------------------------------------

/// 最小二乘去线性趋势：拟合 y = a·n + b，然后减去。
static void removeTrend(std::vector<double>& v) {
    int N = static_cast<int>(v.size());
    if (N < 2) return;
    double sum_n = 0.0, sum_v = 0.0, sum_nv = 0.0, sum_n2 = 0.0;
    for (int i = 0; i < N; ++i) {
        sum_n  += i;
        sum_v  += v[i];
        sum_nv += static_cast<double>(i) * v[i];
        sum_n2 += static_cast<double>(i) * i;
    }
    double denom = static_cast<double>(N) * sum_n2 - sum_n * sum_n;
    if (std::abs(denom) < 1e-12) return;
    double a = (static_cast<double>(N) * sum_nv - sum_n * sum_v) / denom;
    double b = (sum_v - a * sum_n) / N;
    for (int i = 0; i < N; ++i) v[i] -= a * i + b;
}

/// 减去均值后除以标准差归一化。若标准差约为 0 则返回 false。
static bool demeanNormalize(std::vector<double>& v) {
    double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    for (auto& x : v) x -= mean;
    double sq = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double sd = std::sqrt(sq / v.size());
    if (sd < 1e-9) return false;
    for (auto& x : v) x /= sd;
    return true;
}

// ---------------------------------------------------------------------------
// TimeDomainCalc::compute 实现
// ---------------------------------------------------------------------------

CoreResult TimeDomainCalc::compute(
    std::vector<double>& xn,
    std::vector<double>& yn,
    int    N,
    double resample_hz,
    double max_offset_ms,
    bool   detrend)
{
    CoreResult res{};
    res.valid = false;

    // --- 1. 预处理 ---
    if (detrend) {
        removeTrend(xn);
        removeTrend(yn);
    }
    if (!demeanNormalize(xn)) {
        res.message = "camera signal has near-zero variance (insufficient motion?)";
        return res;
    }
    if (!demeanNormalize(yn)) {
        res.message = "IMU signal has near-zero variance (insufficient motion?)";
        return res;
    }

    // --- 2. 计算互相关 ---
    int M = static_cast<int>(max_offset_ms * resample_hz / 1000.0);
    M = std::min(M, N - 1);

    std::vector<double> xcorr(2 * M + 1, 0.0);
    for (int mi = -M; mi <= M; ++mi) {
        double s   = 0.0;
        int    cnt = 0;
        for (int n = 0; n < N; ++n) {
            int ny = n + mi;
            if (ny >= 0 && ny < N) {
                s += xn[n] * yn[ny];
                ++cnt;
            }
        }
        xcorr[mi + M] = (cnt > 0) ? (s / cnt) : 0.0;
    }

    // --- 3. 找峰值 ---
    int    peak_idx = 0;
    double peak_abs = 0.0;
    for (int i = 0; i < static_cast<int>(xcorr.size()); ++i) {
        if (std::abs(xcorr[i]) > peak_abs) {
            peak_abs = std::abs(xcorr[i]);
            peak_idx = i;
        }
    }

    // --- 4. 计算旁瓣均方根 ---
    double sq_side  = 0.0;
    int    cnt_side = 0;
    for (int i = 0; i < static_cast<int>(xcorr.size()); ++i) {
        if (std::abs(i - peak_idx) > 2) {
            sq_side += xcorr[i] * xcorr[i];
            ++cnt_side;
        }
    }
    double rms_side = (cnt_side > 0) ? std::sqrt(sq_side / cnt_side) : 1.0;

    // --- 5. 输出结果 ---
    res.tau        = static_cast<double>(peak_idx - M) / resample_hz;
    res.peak_value = xcorr[peak_idx];
    res.peak_ratio = (rms_side > 1e-12) ? peak_abs / rms_side : 0.0;
    res.valid      = true;
    res.message    = "ok";
    return res;
}
