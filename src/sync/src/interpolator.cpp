#include "sync/interpolator.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

// ---------------------------------------------------------------------------
// 工具：辅助函数
// ---------------------------------------------------------------------------

/// 在有序数组 t 中用二分查找找到最大的下标 j 使得 t[j] <= tq。
/// 假设 t.size() >= 2，且 tq 在 t 范围内（调用侧保证）。
static size_t lowerBound(const std::vector<double>& t, double tq)
{
    size_t lo = 0, hi = t.size() - 2;
    while (lo < hi) {
        size_t mid = (lo + hi + 1) / 2;
        if (t[mid] <= tq) lo = mid;
        else              hi = mid - 1;
    }
    return lo;
}

// ---------------------------------------------------------------------------
// InterpMethod 工具函数
// ---------------------------------------------------------------------------

InterpMethod interpMethodFromString(const std::string& s)
{
    std::string low;
    low.reserve(s.size());
    for (char c : s) low += static_cast<char>(std::tolower(c));

    if (low == "zero_order" || low == "zero-order" || low == "zoh")
        return InterpMethod::ZERO_ORDER;
    if (low == "linear")
        return InterpMethod::LINEAR;
    if (low == "cubic_spline" || low == "cubic-spline" || low == "spline")
        return InterpMethod::CUBIC_SPLINE;
    if (low == "hermite" || low == "pchip")
        return InterpMethod::HERMITE;
    return InterpMethod::LINEAR;  // 默认
}

const char* interpMethodName(InterpMethod m)
{
    switch (m) {
        case InterpMethod::ZERO_ORDER:   return "zero_order";
        case InterpMethod::LINEAR:       return "linear";
        case InterpMethod::CUBIC_SPLINE: return "cubic_spline";
        case InterpMethod::HERMITE:      return "hermite";
        default:                         return "unknown";
    }
}

// ---------------------------------------------------------------------------
// 零阶保持插值
// ---------------------------------------------------------------------------

std::vector<double> ZeroOrderInterp::interp(
    const std::vector<double>& t_src,
    const std::vector<double>& v_src,
    const std::vector<double>& t_grid) const
{
    std::vector<double> out;
    out.reserve(t_grid.size());

    for (double tq : t_grid) {
        if (tq <= t_src.front()) {
            out.push_back(v_src.front());
            continue;
        }
        if (tq >= t_src.back()) {
            out.push_back(v_src.back());
            continue;
        }
        size_t j = lowerBound(t_src, tq);
        out.push_back(v_src[j]);
    }
    return out;
}

// ---------------------------------------------------------------------------
// 分段线性插值
// ---------------------------------------------------------------------------

std::vector<double> LinearInterp::interp(
    const std::vector<double>& t_src,
    const std::vector<double>& v_src,
    const std::vector<double>& t_grid) const
{
    std::vector<double> out;
    out.reserve(t_grid.size());

    for (double tq : t_grid) {
        if (tq <= t_src.front()) {
            out.push_back(v_src.front());
            continue;
        }
        if (tq >= t_src.back()) {
            out.push_back(v_src.back());
            continue;
        }
        size_t j = lowerBound(t_src, tq);
        double dt = t_src[j + 1] - t_src[j];
        if (dt < 1e-12) {
            out.push_back(v_src[j]);
            continue;
        }
        double alpha = (tq - t_src[j]) / dt;
        out.push_back(v_src[j] * (1.0 - alpha) + v_src[j + 1] * alpha);
    }
    return out;
}

// ---------------------------------------------------------------------------
// 自然三次样条插值
// ---------------------------------------------------------------------------
//
// 算法：
//   令 h[i] = t[i+1] - t[i]，M[i] 为第 i 个节点处的二阶导数。
//   自然端点条件：M[0] = M[n-1] = 0。
//   内部节点满足三对角方程（Thomas 算法求解）：
//     h[i]*M[i-1] + 2*(h[i]+h[i+1])*M[i] + h[i+1]*M[i+1]
//         = 6 * ((v[i+1]-v[i])/h[i+1] - (v[i]-v[i-1])/h[i])
//   插值公式（在区间 [t[j], t[j+1]] 内）：
//     A = (t[j+1]-tq)/h[j]，B = 1-A
//     s = A*v[j] + B*v[j+1] + ((A³-A)*M[j] + (B³-B)*M[j+1]) * h[j]² / 6
// ---------------------------------------------------------------------------

std::vector<double> CubicSplineInterp::interp(
    const std::vector<double>& t_src,
    const std::vector<double>& v_src,
    const std::vector<double>& t_grid) const
{
    const size_t n = t_src.size();

    // --- 计算节点间距 ---
    std::vector<double> h(n - 1);
    for (size_t i = 0; i < n - 1; ++i) h[i] = t_src[i + 1] - t_src[i];

    // --- Thomas 算法求解二阶导数 M[1..n-2]（M[0]=M[n-1]=0）---
    std::vector<double> M(n, 0.0);

    if (n >= 3) {
        const size_t m = n - 2;  // 内部节点数量
        std::vector<double> diag(m), rhs(m), c_prime(m), d_prime(m);

        for (size_t k = 0; k < m; ++k) {
            size_t i = k + 1;  // 对应原始节点索引
            diag[k] = 2.0 * (h[i - 1] + h[i]);
            rhs[k]  = 6.0 * ((v_src[i + 1] - v_src[i]) / h[i]
                            - (v_src[i] - v_src[i - 1]) / h[i - 1]);
        }

        // 子对角线/超对角线元素（k 对应 h[i]，即 h[k+1]）
        // 前向消元
        c_prime[0] = h[1] / diag[0];
        d_prime[0] = rhs[0] / diag[0];
        for (size_t k = 1; k < m; ++k) {
            double denom = diag[k] - h[k] * c_prime[k - 1];
            c_prime[k]  = (k + 1 < m) ? h[k + 1] / denom : 0.0;
            d_prime[k]  = (rhs[k] - h[k] * d_prime[k - 1]) / denom;
        }

        // 回代
        M[n - 2] = d_prime[m - 1];
        for (int k = static_cast<int>(m) - 2; k >= 0; --k) {
            M[k + 1] = d_prime[k] - c_prime[k] * M[k + 2];
        }
    }

    // --- 逐点插值 ---
    std::vector<double> out;
    out.reserve(t_grid.size());

    for (double tq : t_grid) {
        if (tq <= t_src.front()) {
            out.push_back(v_src.front());
            continue;
        }
        if (tq >= t_src.back()) {
            out.push_back(v_src.back());
            continue;
        }
        size_t j = lowerBound(t_src, tq);
        double hj = h[j];
        double A  = (t_src[j + 1] - tq) / hj;
        double B  = 1.0 - A;
        double val = A * v_src[j] + B * v_src[j + 1]
                   + ((A * A * A - A) * M[j] + (B * B * B - B) * M[j + 1])
                     * hj * hj / 6.0;
        out.push_back(val);
    }
    return out;
}

// ---------------------------------------------------------------------------
// 分段三次 Hermite 插值（PCHIP）
// ---------------------------------------------------------------------------
//
// 斜率估计（Fritsch-Carlson）：
//   delta[i] = (v[i+1]-v[i]) / h[i]
//   两端：d[0]=delta[0]，d[n-1]=delta[n-2]
//   内部点 i：若 delta[i-1]*delta[i] <= 0 则 d[i]=0（局部极值）
//             否则调和平均：
//               w1=2h[i]+h[i-1]，w2=h[i]+2h[i-1]
//               1/d[i] = w1/(（w1+w2）*delta[i-1]) + w2/(（w1+w2）*delta[i])
//
// Hermite 基函数（局部坐标 t∈[0,1]）：
//   h00 = 2t³-3t²+1，h10 = t³-2t²+t，h01 = -2t³+3t²，h11 = t³-t²
//   p(t) = h00*v[j] + h10*H*d[j] + h01*v[j+1] + h11*H*d[j+1]  （H=h[j]）
// ---------------------------------------------------------------------------

std::vector<double> HermiteInterp::interp(
    const std::vector<double>& t_src,
    const std::vector<double>& v_src,
    const std::vector<double>& t_grid) const
{
    const size_t n = t_src.size();

    // --- 节点间距与斜率 ---
    std::vector<double> h(n - 1), delta(n - 1);
    for (size_t i = 0; i < n - 1; ++i) {
        h[i]     = t_src[i + 1] - t_src[i];
        delta[i] = (h[i] > 1e-12) ? (v_src[i + 1] - v_src[i]) / h[i] : 0.0;
    }

    std::vector<double> d(n);
    d[0]     = delta[0];
    d[n - 1] = delta[n - 2];
    for (size_t i = 1; i < n - 1; ++i) {
        if (delta[i - 1] * delta[i] <= 0.0) {
            d[i] = 0.0;
        } else {
            double w1   = 2.0 * h[i] + h[i - 1];
            double w2   = h[i] + 2.0 * h[i - 1];
            double wsum = w1 + w2;
            d[i] = wsum / (w1 / delta[i - 1] + w2 / delta[i]);
        }
    }

    // --- 逐点插值 ---
    std::vector<double> out;
    out.reserve(t_grid.size());

    for (double tq : t_grid) {
        if (tq <= t_src.front()) {
            out.push_back(v_src.front());
            continue;
        }
        if (tq >= t_src.back()) {
            out.push_back(v_src.back());
            continue;
        }
        size_t j = lowerBound(t_src, tq);
        double H  = h[j];
        double tt = (tq - t_src[j]) / H;  // 局部坐标 ∈ [0,1]
        double tt2 = tt * tt;
        double tt3 = tt2 * tt;

        double h00 =  2.0 * tt3 - 3.0 * tt2 + 1.0;
        double h10 =        tt3 - 2.0 * tt2 + tt;
        double h01 = -2.0 * tt3 + 3.0 * tt2;
        double h11 =        tt3 -       tt2;

        out.push_back(h00 * v_src[j] + h10 * H * d[j]
                    + h01 * v_src[j + 1] + h11 * H * d[j + 1]);
    }
    return out;
}

// ---------------------------------------------------------------------------
// 工厂函数
// ---------------------------------------------------------------------------

std::unique_ptr<InterpBase> makeInterpolator(InterpMethod method)
{
    switch (method) {
        case InterpMethod::ZERO_ORDER:   return std::make_unique<ZeroOrderInterp>();
        case InterpMethod::LINEAR:       return std::make_unique<LinearInterp>();
        case InterpMethod::CUBIC_SPLINE: return std::make_unique<CubicSplineInterp>();
        case InterpMethod::HERMITE:      return std::make_unique<HermiteInterp>();
        default:                         return std::make_unique<LinearInterp>();
    }
}
