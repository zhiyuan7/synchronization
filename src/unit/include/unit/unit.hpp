#pragma once


namespace unit::literals {

    constexpr double UNIT_PI = 3.1415926535897932384626433832795;

    // 声明"弧度"的存在
    struct radian;

    /**
    * @brief 角度结构体
    * 
    */
    struct degree {
        float value;

        explicit constexpr degree(float val) : value(val) {}
        explicit constexpr degree(const radian& val);
    } __attribute__((packed));

    /**
    * @brief 弧度结构体
    * 
    */
    struct radian {
        float value;

        explicit constexpr radian(float val) : value(val) {}
        explicit constexpr radian(const degree& deg) : value(deg.value * UNIT_PI / 180.0) {}
    } __attribute__((packed));

    constexpr degree::degree(const radian& val) : value(val.value * 180.0 / UNIT_PI) {}

    constexpr degree operator"" _deg(long double val) {
        return degree(static_cast<float>(val));
    }

    constexpr degree operator"" _deg(unsigned long long val) {
        return degree(static_cast<float>(val));
    }

    constexpr radian operator"" _rad(long double val) {
        return radian(static_cast<float>(val));
    }

    constexpr radian operator"" _rad(unsigned long long val) {
        return radian(static_cast<float>(val));
    }

    constexpr degree operator+(const degree& l, const degree& r) { return degree(l.value + r.value); }
    constexpr degree operator-(const degree& l, const degree& r) { return degree(l.value - r.value); }
    constexpr degree operator-(const degree& d) { return degree(-d.value); }
    constexpr degree operator*(const degree& l, float scaler) { return degree(l.value * scaler); }
    constexpr degree operator*(float scaler, const degree& r) { return degree(scaler * r.value); }
    constexpr degree operator/(const degree& l, float scaler) { return degree(l.value / scaler); }
    constexpr degree operator/(float scaler, const degree& r) { return degree(scaler / r.value); }

    constexpr radian operator+(const radian& l, const radian& r) { return radian(l.value + r.value); }
    constexpr radian operator-(const radian& l, const radian& r) { return radian(l.value - r.value); }
    constexpr radian operator-(const radian& r) { return radian(-r.value); }
    constexpr radian operator*(const radian& l, float scaler) { return radian(l.value * scaler); }
    constexpr radian operator*(float scaler, const radian& r) { return radian(scaler * r.value); }
    constexpr radian operator/(const radian& l, float scaler) { return radian(l.value / scaler); }
    constexpr radian operator/(float scaler, const radian& r) { return radian(scaler / r.value); }

    constexpr degree operator+(const degree& l, const radian& r) { return l + degree(r); }
    constexpr degree operator+(const radian& l, const degree& r) { return degree(l) + r; }

    // degree 与 double 算术运算
    constexpr degree operator+(const degree& l, double r) { return degree(l.value + r); }
    constexpr degree operator+(double l, const degree& r) { return degree(l + r.value); }
    constexpr degree operator-(const degree& l, double r) { return degree(l.value - r); }
    constexpr degree operator-(double l, const degree& r) { return degree(l - r.value); }
    constexpr degree operator*(const degree& l, double scaler) { return degree(l.value * scaler); }
    constexpr degree operator*(double scaler, const degree& r) { return degree(scaler * r.value); }
    constexpr degree operator/(const degree& l, double scaler) { return degree(l.value / scaler); }
    constexpr degree operator/(double scaler, const degree& r) { return degree(scaler / r.value); }

    // radian 与 double 算术运算
    constexpr radian operator+(const radian& l, double r) { return radian(l.value + r); }
    constexpr radian operator+(double l, const radian& r) { return radian(l + r.value); }
    constexpr radian operator-(const radian& l, double r) { return radian(l.value - r); }
    constexpr radian operator-(double l, const radian& r) { return radian(l - r.value); }
    constexpr radian operator*(const radian& l, double scaler) { return radian(l.value * scaler); }
    constexpr radian operator*(double scaler, const radian& r) { return radian(scaler * r.value); }
    constexpr radian operator/(const radian& l, double scaler) { return radian(l.value / scaler); }
    constexpr radian operator/(double scaler, const radian& r) { return radian(scaler / r.value); }

    constexpr bool operator==(const degree& l, const degree& r) { return l.value == r.value; }
    constexpr bool operator<(const degree& l, const degree& r) { return l.value < r.value; }
    constexpr bool operator<=(const degree& l, const degree& r) { return l.value <= r.value; }
    constexpr bool operator>(const degree& l, const degree& r) { return l.value > r.value; }
    constexpr bool operator>=(const degree& l, const degree& r) { return l.value >= r.value; }

    constexpr bool operator==(const radian& l, const radian& r) { return l.value == r.value; }
    constexpr bool operator<(const radian& l, const radian& r) { return l.value < r.value; }
    constexpr bool operator<=(const radian& l, const radian& r) { return l.value <= r.value; }
    constexpr bool operator>(const radian& l, const radian& r) { return l.value > r.value; }
    constexpr bool operator>=(const radian& l, const radian& r) { return l.value >= r.value; }

    // degree 与 double 比较
    constexpr bool operator==(const degree& l, double r) { return l.value == r; }
    constexpr bool operator==(double l, const degree& r) { return l == r.value; }
    constexpr bool operator<(const degree& l, double r) { return l.value < r; }
    constexpr bool operator<(double l, const degree& r) { return l < r.value; }
    constexpr bool operator<=(const degree& l, double r) { return l.value <= r; }
    constexpr bool operator<=(double l, const degree& r) { return l <= r.value; }
    constexpr bool operator>(const degree& l, double r) { return l.value > r; }
    constexpr bool operator>(double l, const degree& r) { return l > r.value; }
    constexpr bool operator>=(const degree& l, double r) { return l.value >= r; }
    constexpr bool operator>=(double l, const degree& r) { return l >= r.value; }

    // radian 与 double 比较
    constexpr bool operator==(const radian& l, double r) { return l.value == r; }
    constexpr bool operator==(double l, const radian& r) { return l == r.value; }
    constexpr bool operator<(const radian& l, double r) { return l.value < r; }
    constexpr bool operator<(double l, const radian& r) { return l < r.value; }
    constexpr bool operator<=(const radian& l, double r) { return l.value <= r; }
    constexpr bool operator<=(double l, const radian& r) { return l <= r.value; }
    constexpr bool operator>(const radian& l, double r) { return l.value > r; }
    constexpr bool operator>(double l, const radian& r) { return l > r.value; }
    constexpr bool operator>=(const radian& l, double r) { return l.value >= r; }
    constexpr bool operator>=(double l, const radian& r) { return l >= r.value; }

}

enum class TeamColor : unsigned char {
    RED = 0,
    BLUE = 1,
};

enum class ProcessState : unsigned char {
    NORMAL_FILTER = 0,      // 时间间隔合适，这一次识别到了 → 正常滤波
    PREDICT_ONLY = 1,       // 时间间隔合适，这一次未识别到 → 仅仅是按照原有状态进行预测，不进行滤波
    RESET_FILTER = 2,       // 时间间隔过长，这一次识别到了 → 重置滤波器状态
    DISCARD = 3             // 时间间隔过长，这一次未识别到 → 丢弃
};

enum RobotTypeEnum {
    NORMAL = 0, // 4 块小装甲板
    HERO = 1,   // 英雄，4 块大装甲板
};

/**
 * 浅谈一下 Tracker 的索敌机制
 * 
 * 一个合格的 Tracker 应该实现以下几点功能:
 * 1. 当视野中没有目标时，等待目标出现。视野中一旦发现目标，立刻锁定最近的那个
 * 2. 操作手按住右键 (is_change = 1)，此时表示操作手希望一直锁定这辆车，即使丢失目标以后也不能更换
 *    比如正在集火步兵，一辆工程档了一下，不锁工程，等工程走开后接着锁步兵
 * 3. 操作手没有按右键锁某一辆车，则目标丢失一段时间以后，重新回到 1.
 * 
 * 由上面的基本逻辑，可以引出下面的具体实现:
 * - Searching  || 没有任何目标，正在搜索状态
 * - Tracking   || 锁定目标状态
 * - Lost       || 目标丢失，但为操作手保留正在锁定的 id
 * 
 * 1. Searching -> Tracking | 识别到任意车辆
 * 2. Tracking -> Tracking  | 识别到锁定的车辆
 * 3. Tracking -> Searching | 无识别 & 无锁定
 * 4. Tracking -> Lost      | 无识别 & 有锁定
 * 5. Lost -> Searching     | 丢失时间过长
 * 6. Lost -> Tracking      | 识别到锁定的车辆
 */
 enum class TrackingState : unsigned char {
    Searching = 0, // 未识别到目标
    Tracking = 1,  // 正在追踪目标
    Lost = 2       // 目标丢失
};

enum class TrackingMethod : unsigned char { // 跟踪逻辑
    Whole = 0,
    Single = 1
};

enum class WholeShootMethod : unsigned char { // 整车射击逻辑
    Follow = 0, // 转速慢, 跟随打
    Center = 1, // 转速中高, 瞄中心, 预测发射时机
    Fury = 2    // 转速很高, 瞄中心, 始终射击
};