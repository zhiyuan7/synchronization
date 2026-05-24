#pragma once

#include <opencv2/core/types.hpp>
#include <vector>

#include <opencv2/opencv.hpp>

#include "unit/unit.hpp"

// ---------------------------------------
// ----------------mode-------------------
// ---------------------------------------

// 兵种模式
enum ModeEnum{
    ENUM_MODE_ARMOR = 0,
    ENUM_MODE_INFANTRY_LITTLE_BUFF = 1,
    ENUM_MODE_INFANTRY_BIG_BUFF = 2,
};

// ---------------------------------------
// ----------------inference----------------
// ---------------------------------------
struct SingleInfer {
    int class_id;
    std::vector<cv::Point2f> keypoints;
    float confidence; // TODO: maybe useless?
};

using CoupleInfers = std::vector<SingleInfer>;


// ---------------------------------------
// ----------------detetct----------------
// ---------------------------------------

// 装甲板检测结果
struct SingleDetection {
    // keypoints
    // 1*           *4
    //  |           |
    //  |           |
    // 2*-----------*3
    std::vector<cv::Point2f> kpts;
    bool is_big; // 是否是大装甲板
    TeamColor color; // 0 red, 1 blue
    int id; // 0: guard, (1, 2, 3, 4, 5, ) 6: outpost, 7: base
};

using CoupleDetections = std::vector<SingleDetection>;


// -------------------------------------
// -----------------灯条-----------------
// -------------------------------------

// 定义直线参数结构体  
struct LinePara {
    double k;
	double b;
};

bool getCrossPoint(
    const cv::Point2f &p1, const cv::Point2f &p2,
    const cv::Point2f &p3, const cv::Point2f &p4,
    cv::Point2f &cross_point
);

struct Light {

public:
    Light(cv::Point2f top, cv::Point2f bottom); // 给Mix用的
    Light(const std::vector<cv::Point> &cnt);
    bool operator==(const Light &other) const;

    cv::Point2f top;
    cv::Point2f bottom;
    TeamColor color; // 0 red, 1 blue

    // 用于筛选
    cv::Point2f center;
    cv::Rect bounding_rect;
    double length;
    double width;

    // 用于修正端点
    std::vector<cv::Point> contour;

private:
    std::vector<cv::Point2f> getLightPointsByMinAreaRect(cv::RotatedRect rotate_rect);
    std::vector<cv::Point2f> getLightPointsByEllipse(cv::RotatedRect ellipse, cv::RotatedRect rotate_rect);

};

using PairLights = std::pair<Light, Light>;
using CoupleLights = std::vector<Light>;
using CouplePairLights = std::vector<PairLights>;

// -------------------------------------
// -----------------pose----------------
// -------------------------------------

// 坐标
struct SinglePos {
    cv::Point3d camera; // 相机系坐标
    cv::Point3d world;  // 世界系坐标
    double yaw;         // 装甲板世界系yaw
    int id;             // 装甲板id
    double pnp_reproj_err = 0.0; // pnp重投影误差
};

using CouplePos = std::vector<SinglePos>;

// 装甲板真实世界大小, 用于solvePnP
struct ArmorRealSize {
    std::vector<cv::Point3d> small = {
        cv::Point3d(0, 67.5, 27.5), cv::Point3d(0, 67.5, -27.5),
        cv::Point3d(0, -67.5, -27.5), cv::Point3d(0, -67.5, 27.5) 
    };
    std::vector<cv::Point3d> big = {
        cv::Point3d(0, 112.5, 27.5), cv::Point3d(0, 112.5, -27.5),
        cv::Point3d(0, -112.5, -27.5), cv::Point3d(0, -112.5, 27.5)
    };
	std::vector<cv::Point3d> outpost = {
		cv::Point3d(0,64.5,27.5),cv::Point3d(0,64.5,-27.5),
		cv::Point3d(0,-64.5,-27.5),cv::Point3d(0,-64.5,27.5)
	};
};

// -------------------------------------
// ----------------filter---------------
// -------------------------------------

struct SingleVehicle {
    int id; // 车的id
    bool is_big; // 是否是大装甲板
    int armor_num; // 有几块装甲板 {2，3，4}
    cv::Point3d center; // 车中心世界坐标
    cv::Point3d center_vel; // 车中心世界坐标速度
    double yaw; // 车 yaw角
    double vyaw; // 车 yaw角速度
    double ayaw; // 车 yaw角加速度
    double ra; // yaw 对应的半径
    double rb; // yaw + PI/2 对应的半径
    double dh; // yaw 对应装甲板高度就是 center.y, yaw + PI/2 对应装甲板高度是 center.y + dh
    bool is_reset = false; // 本帧是否触发了滤波器重置
};

using CoupleVehicles = std::vector<SingleVehicle>;

struct SingleArmor {
    int id; // 装甲板id
    cv::Point3d pos; // 装甲板世界坐标系的坐标
    cv::Point3d vel; // xz 速度
    cv::Point3d acce; // xz 加速度
    bool is_big; // 是否为大装甲板
};

using CoupleArmors = std::vector<SingleArmor>;

// -------------------------------------
// ----------------track----------------
// -------------------------------------

// 跟踪结果
/// 某一相对时刻 t 下的装甲板几何状态（VehicleTrack / yaw MPC 共用）
struct ArmorGeom {
    cv::Point3d center_t;      ///< t 时刻整车中心位置
    double vehicle_yaw_t;      ///< t 时刻整车 yaw (rad)
    double armor_yaw_world;    ///< t 时刻目标装甲板在世界系中的 yaw (rad)
    double r;                  ///< 所选装甲板到中心的半径
    double dz;                 ///< 所选装甲板相对 center 的 z 偏移
    cv::Point3d armor_pos;     ///< t 时刻装甲板 3D 位置
};

struct TrackResult {
    bool is_get;          // 是否获取到目标
    bool is_shoot;        // 是否射击
    int id;               // 目标ID
    double yaw;          // 偏航角
    double hope_yaw;     // 未加前馈的期望偏航角
    double pitch;        // 俯仰角
    double shoot_yaw;    // 射击时偏航角(相机到世界坐标系)
    double shoot_pitch;  // 射击时俯仰角(相机到世界坐标系)
    double aim_x;        // 目标x坐标(世界坐标系)
    double aim_y;        // 目标y坐标(世界坐标系)
    double aim_z;        // 目标z坐标(世界坐标系)
    double predict_x;    // 预测x坐标(世界坐标系)
    double predict_y;    // 预测y坐标(世界坐标系)
    double predict_z;    // 预测z坐标(世界坐标系)
    double yaw_velocity;      // yaw角速度(rad/s)
    double yaw_acceleration;  // yaw角加速度(rad/s²)
	bool mpc_is_shoot;   //mpc判断是否击打
};

// -------------------------------------
// ----------------buff-----------------
// -------------------------------------

// Buff 检测结果（图像域）
struct BuffDetection {
    std::vector<cv::Point2f> buffs; // 扇叶中心点图像坐标，主要用于坐标解算;如果为空，则没识别到
    cv::Point2f center; // 旋转中心点图像坐标
    std::vector<double> target_angle; // 目标扇叶角度数组

    double radius; // 由 Detector 算出来，识别的时候不用赋值
};

// Buff 滤波后的角度结果
struct AngleFilter {
    bool valid; // 是否有效
    bool jump; // 和上一帧相比是否跳变
    double angle; // 角度
    double speed; // 角速度
};

// Buff 预测函数类型
typedef std::function<double(int64_t)> AnglePredFunc;

// Buff 滤波结果
struct BuffFilter {
    cv::Point3d center;         // 中心点世界坐标

    // 角度滤波结果（内部使用）
    AngleFilter angle_filter;
    std::vector<int32_t> nsteps; // 当前帧每个角度吸附对应的步数
    
    // 拟合参数
    double amplitude;           // 振幅 a
    double omega;               // 角频率 ω
    double phase;               // 相位 φ
    double bias;                // 偏置 b
    int64_t last_time;          // 上次更新时间
    int64_t base_time;          // 基准时间
    double last_angle;          // 上次角度
};
