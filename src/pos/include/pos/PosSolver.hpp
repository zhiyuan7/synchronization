#pragma once

#include "Converter.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "unit/unit.hpp"
#include "unit/types.hpp"
#include <vector>

using namespace unit::literals;

class PosSolver {

public:
    PosSolver() = default;
    PosSolver(const std::string cam_param, const std::string offset_param);
    ~PosSolver() = default;

    void updateStmAndStamp(const rclcpp::Time stamp);

    CouplePos getPosition(const CoupleDetections &detections);

    std::shared_ptr<Converter> converter;

    double* c_camera_intrisic;
    std::array<double, 3> euler_offset_deg= {0.0, 0.0, 0.0}; // [yaw, pitch, roll]（度）
    std::array<double, 9> cam_intrinsic_cached;
    std::vector<double> cam_distortion_cached;

private:
    ArmorRealSize real_size;

    double fdb_yaw = 0.0; // 反馈回来的yaw, 用于算yaw
    double c_yaw_offset = 0.0;
    double c_pitch_offset = 0.0;
    double c_roll_offset = 0.0;
    int yaw_mode = 0; // 0: pnp_yaw, 1: first_yaw, 2: second_yaw
};