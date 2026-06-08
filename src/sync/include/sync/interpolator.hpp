#pragma once

#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// 插值方法枚举
// ---------------------------------------------------------------------------
enum class InterpMethod {
    ZERO_ORDER,   ///< 零阶保持（ZOH）：取前驱采样值，不引入插值误差
    LINEAR,       ///< 分段线性插值：简单快速，C⁰连续
    CUBIC_SPLINE, ///< 自然三次样条：C²连续，全局平滑，但可能轻微过冲
    HERMITE       ///< 分段三次 Hermite（PCHIP）：保持单调性、无过冲，适合传感器数据
};

/// 从字符串解析 InterpMethod（不区分大小写）。
/// 支持: "zero_order" | "linear" | "cubic_spline" | "hermite"
/// 未知字符串默认返回 LINEAR。
InterpMethod interpMethodFromString(const std::string& s);

/// 返回 InterpMethod 对应的可读名称（用于日志）
const char* interpMethodName(InterpMethod m);

// ---------------------------------------------------------------------------
// 插值器抽象基类
// ---------------------------------------------------------------------------
class InterpBase {
public:
    virtual ~InterpBase() = default;

    /// 将非均匀采样信号插值到均匀网格上。
    ///
    /// @param t_src   源时间戳，必须严格升序，长度 >= 2
    /// @param v_src   源信号值，长度与 t_src 相同
    /// @param t_grid  目标时间网格，必须升序
    /// @return        与 t_grid 等长的插值结果；超出 [t_src.front(), t_src.back()]
    ///                的查询点截断到最近边界值
    virtual std::vector<double> interp(
        const std::vector<double>& t_src,
        const std::vector<double>& v_src,
        const std::vector<double>& t_grid) const = 0;
};

// ---------------------------------------------------------------------------
// 零阶保持插值
// ---------------------------------------------------------------------------
class ZeroOrderInterp : public InterpBase {
public:
    std::vector<double> interp(
        const std::vector<double>& t_src,
        const std::vector<double>& v_src,
        const std::vector<double>& t_grid) const override;
};

// ---------------------------------------------------------------------------
// 分段线性插值
// ---------------------------------------------------------------------------
class LinearInterp : public InterpBase {
public:
    std::vector<double> interp(
        const std::vector<double>& t_src,
        const std::vector<double>& v_src,
        const std::vector<double>& t_grid) const override;
};

// ---------------------------------------------------------------------------
// 自然三次样条插值（端部二阶导数为零）
// ---------------------------------------------------------------------------
class CubicSplineInterp : public InterpBase {
public:
    std::vector<double> interp(
        const std::vector<double>& t_src,
        const std::vector<double>& v_src,
        const std::vector<double>& t_grid) const override;
};

// ---------------------------------------------------------------------------
// 分段三次 Hermite 插值（PCHIP，Fritsch-Carlson 斜率估计）
// ---------------------------------------------------------------------------
class HermiteInterp : public InterpBase {
public:
    std::vector<double> interp(
        const std::vector<double>& t_src,
        const std::vector<double>& v_src,
        const std::vector<double>& t_grid) const override;
};

// ---------------------------------------------------------------------------
// 工厂函数
// ---------------------------------------------------------------------------
std::unique_ptr<InterpBase> makeInterpolator(InterpMethod method);
