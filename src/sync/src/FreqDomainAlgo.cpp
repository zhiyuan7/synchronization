#include "sync/FreqDomainAlgo.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <opencv2/core.hpp>

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

/// 汉宁窗：w[n] = 0.5·(1 − cos(2π·n/(N−1)))，原地乘入信号。
static void applyHann(std::vector<double>& v) {
    int N = static_cast<int>(v.size());
    if (N < 2) return;
    for (int i = 0; i < N; ++i) {
        double w = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (N - 1)));
        v[i] *= w;
    }
}

// ---------------------------------------------------------------------------
// FreqDomainCalc 构造函数
// ---------------------------------------------------------------------------

FreqDomainCalc::FreqDomainCalc(bool apply_window, double max_fit_hz,
                               const std::string& freq_fit_mode, int fixed_bins_count)
    : apply_window_(apply_window), max_fit_hz_(max_fit_hz),
      use_fixed_bins_(freq_fit_mode == "fixed_bins"),
      fixed_bins_count_(fixed_bins_count)
{}

// ---------------------------------------------------------------------------
// FreqDomainCalc::compute 实现
// ---------------------------------------------------------------------------

CoreResult FreqDomainCalc::compute(
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

    // --- 2. 加窗（汉宁窗，降低截断谱泄漏） ---
    if (apply_window_) {
        applyHann(xn);
        applyHann(yn);
    }

    // --- 3. 补零到 L = nextpow2(N) ---
    int L = 1;
    while (L < N) L <<= 1;

    // --- 4. FFT（OpenCV DFT，复数输出） ---
    cv::Mat Xm = cv::Mat::zeros(1, L, CV_64F);
    cv::Mat Ym = cv::Mat::zeros(1, L, CV_64F);
    for (int i = 0; i < N; ++i) {
        Xm.at<double>(0, i) = xn[i];
        Ym.at<double>(0, i) = yn[i];
    }
    cv::Mat Xf, Yf;
    cv::dft(Xm, Xf, cv::DFT_COMPLEX_OUTPUT);
    cv::dft(Ym, Yf, cv::DFT_COMPLEX_OUTPUT);

    // --- 5. 互功率谱 S_xy[k] = conj(X[k])·Y[k]，取正频段 k ∈ [1, L/2−1] ---
    // conj(X)·Y = (xr − j·xi)(yr + j·yi) = (xr·yr + xi·yi) + j·(xr·yi − xi·yr)
    int K = L / 2 - 1;  // 正频点数（去除 DC k=0 与 Nyquist k=L/2）
    if (K < 2) {
        res.message = "too few FFT bins for phase fitting";
        return res;
    }

    std::vector<double> omega(K), phi(K);
    for (int i = 0; i < K; ++i) {
        int k = i + 1;
        const cv::Vec2d& xv = Xf.at<cv::Vec2d>(0, k);
        const cv::Vec2d& yv = Yf.at<cv::Vec2d>(0, k);
        double sr = xv[0] * yv[0] + xv[1] * yv[1];
        double si = xv[0] * yv[1] - xv[1] * yv[0];
        omega[i] = 2.0 * M_PI * k / static_cast<double>(L);  // 样本归一化角频率
        phi[i]   = std::atan2(si, sr);
    }

    // --- 6. 相位 unwrap（标准累积展开，将 ±2π 跳变展平） ---
    for (int i = 1; i < K; ++i) {
        double diff = phi[i] - phi[i - 1];
        diff -= 2.0 * M_PI * std::round(diff / (2.0 * M_PI));
        phi[i] = phi[i - 1] + diff;
    }

    // --- 7. 最小二乘拟合 φ[k] ≈ a·ω_k + b（cv::solve + DECOMP_SVD） ---
    int K_fit;
    if (use_fixed_bins_) {
        // 固定使用前 fixed_bins_count_ 个频点进行拟合（避免不同采样率下频点数不一致）
        K_fit = std::min(fixed_bins_count_, K);
    } else {
        // 按 max_fit_hz_ 截断：f_k = k·resample_hz/L，最大有效 k 满足 f_k ≤ max_fit_hz_
        K_fit = K;
        if (max_fit_hz_ > 0.0) {
            int k_max = static_cast<int>(max_fit_hz_ * static_cast<double>(L) / resample_hz);
            K_fit = std::min(k_max, K);
        }
    }
    if (K_fit < 2) {
        res.message = "too few frequency bins for phase fitting";
        return res;
    }
    cv::Mat A_mat(K_fit, 2, CV_64F);
    cv::Mat b_mat(K_fit, 1, CV_64F);
    for (int i = 0; i < K_fit; ++i) {
        A_mat.at<double>(i, 0) = omega[i];
        A_mat.at<double>(i, 1) = 1.0;
        b_mat.at<double>(i, 0) = phi[i];
    }
    cv::Mat coeffs;
    if (!cv::solve(A_mat, b_mat, coeffs, cv::DECOMP_SVD)) {
        res.message = "cv::solve failed in phase fitting";
        return res;
    }
    double slope     = coeffs.at<double>(0, 0);
    double intercept = coeffs.at<double>(1, 0);

    // --- 8. 拟合优度 R²（仅在参与拟合的 K_fit 个频点上计算）---
    double sum_p = 0.0;
    for (int i = 0; i < K_fit; ++i) sum_p += phi[i];
    double mean_p = sum_p / static_cast<double>(K_fit);
    double ss_tot = 0.0, ss_res = 0.0;
    for (int i = 0; i < K_fit; ++i) {
        double fitted = slope * omega[i] + intercept;
        ss_tot += (phi[i] - mean_p) * (phi[i] - mean_p);
        ss_res += (phi[i] - fitted)  * (phi[i] - fitted);
    }
    double fit_r2 = (ss_tot > 1e-15) ? (1.0 - ss_res / ss_tot) : 1.0;

    // --- 9. 延时换算 ---
    // 推导（x=相机，y=IMU，x[n]=y[n−d]，相机滞后 d 样本）：
    //   X[k] = Y[k]·e^{−j·ω_k·d}  →  S_xy = X*·Y = |Y|²·e^{+j·ω_k·d}
    //   φ[k] ≈ +ω_k·d  →  slope = d
    //   tau_samples = −slope = −d  （与时域 m_hat = −d 符号一致）
    //   tau_ms = −tau_samples·(1000/hz) = d·(1000/hz) > 0 ↔ 相机滞后
    double tau_samples = -slope;

    // --- 10. 合理性检查（超出搜索半宽时告警，但仍返回结果供调用方判断） ---
    double tau_s    = tau_samples / resample_hz;          // 延时（秒），tau_ms = -tau_s * 1000
    bool   in_range = std::abs(tau_s * 1000.0) <= max_offset_ms;

    res.tau          = tau_s;
    res.fit_r2       = fit_r2;
    res.num_freq_used = K_fit;
    res.valid        = true;
    res.message      = in_range ? "ok"
                                : "warning: estimated tau exceeds max_offset_ms";
    return res;
}
