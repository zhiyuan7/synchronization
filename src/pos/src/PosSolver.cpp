#include "pos/PosSolver.hpp"

#include "utils/utils.hpp"

#include <chrono>
#include <complex>
#include <exception>
#include <iostream>
#include <cmath>
#include <opencv2/core/types.hpp>

// OpenCV相机坐标系(x,y,z) -> 自定义相机坐标系(z,-x,-y)
inline void opencvToCustom(const cv::Mat& tvec, cv::Point3d& custom) {
    custom.x = tvec.at<double>(2);
    custom.y = -tvec.at<double>(0);
    custom.z = -tvec.at<double>(1);
}

PosSolver::PosSolver(const std::string cam_param, const std::string offset_param) {
    converter = std::make_shared<Converter>(cam_param);

    // 加载相机内参
    if (!cam_param.empty()) {
        YAML::Node node = YAML::LoadFile(cam_param);
        cv::Mat intrinsic_mat = cv::Mat_<double>(3, 3);
        for (int i = 0; i < 3; i++) {
            auto row = node["intrinsic"][i].as<std::vector<double>>();
            std::copy(row.begin(), row.end(), intrinsic_mat.ptr<double>(i));
        }

        // 变为一维数组缓存，供CUDA调用
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                cam_intrinsic_cached[i * 3 + j] = intrinsic_mat.at<double>(i, j);
            }
        }
        cam_distortion_cached = node["distortion"].as<std::vector<double>>();
        if (node["yaw_mode"]) {
            yaw_mode = node["yaw_mode"].as<int>();
        }
    }
    
    // 加载euler offset信息
    if (!offset_param.empty()) {
        YAML::Node offset_node = YAML::LoadFile(offset_param);
        auto euler_deg = offset_node["euler_offset"].as<std::vector<double>>();
        euler_offset_deg[0] = euler_deg[0];  // yaw
        euler_offset_deg[1] = euler_deg[1];  // pitch
        euler_offset_deg[2] = euler_deg[2];  // roll
        
        // 转换为弧度存储到c_offset中
        c_yaw_offset = euler_deg[0] * M_PI / 180.0;
        c_pitch_offset = euler_deg[1] * M_PI / 180.0;
        c_roll_offset = euler_deg[2] * M_PI / 180.0;
    }
}

void PosSolver::updateStmAndStamp(const rclcpp::Time stamp) {
    this->converter->updateStamp(stamp);
}

// 判断解算的坐标是否合法
static bool legalPos(SinglePos &a) {
    if (!std::isfinite(a.camera.x) || !std::isfinite(a.camera.y) || !std::isfinite(a.camera.z) ||
        !std::isfinite(a.world.x) || !std::isfinite(a.world.y) || !std::isfinite(a.world.z)) {
        return false;
    }

    // 太近 (<30cm) 的不打
    if (cv::norm(a.world) < 300) {
        return false;
    }

    // 太远 (>12m) 的目标认为是误识别
    if (cv::norm(a.world) > 12000) {
        return false;
    }

    return true;
}

CouplePos PosSolver::getPosition(const CoupleDetections &detections) {
    CouplePos res;
    
    for (SingleDetection detection: detections) {

        SinglePos tmp;
        std::vector<cv::Point3d> detection_points_3D =
            detection.is_big ? real_size.big : real_size.small;
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);

        // 使用 solvePnPGeneric 获取 IPPE 的两个解
        std::vector<cv::Mat> rvecs, tvecs;
        bool pnp_ok = cv::solvePnPGeneric(
            detection_points_3D, detection.kpts,
            converter->getIntrinsic(),
            converter->getDistortion(),
            rvecs, tvecs, false, cv::SOLVEPNP_IPPE
        );
        // 如果没有成功，退回到单解 solvePnP
        if (!pnp_ok || tvecs.empty()) {
            cv::solvePnP(
                detection_points_3D, detection.kpts,
                converter->getIntrinsic(),
                converter->getDistortion(),
                rvec, tvec, false, cv::SOLVEPNP_IPPE
            );
        } else {
            // pnp双解的xyz变化非常小,这里默认第一个进行坐标转换
            tvec = tvecs[0];
        }
        opencvToCustom(tvec, tmp.camera);

        // 世界坐标系
        tmp.world = converter->camera2world(tmp.camera);

        // 计算yaw
        fdb_yaw = 0.0;

        // 计算来自 IPPE 得到的 pnp yaw/pitch 候选解
        struct PnpCandidate { double yaw; double pitch; };
        std::vector<PnpCandidate> pnp_candidates;
        if (!rvecs.empty()) {
            for (size_t ri = 0; ri < rvecs.size() && ri < 2; ++ri) {
                cv::Mat rmat_i;
                cv::Rodrigues(rvecs[ri], rmat_i);
                // yaw: 自定义坐标系 atan2(-nx, nz)
                double n_x = rmat_i.at<double>(0, 0);
                double n_z = rmat_i.at<double>(2, 0);
                double custom_n_x = n_z;
                double custom_n_y = -n_x;
                double cand_yaw = atan2(custom_n_y, custom_n_x);
                // pitch: OpenCV坐标系x轴方向，即旋转矩阵第一列的仰角
                double pitch_x = rmat_i.at<double>(0, 0);
                double pitch_y = rmat_i.at<double>(1, 0);
                double pitch_z = rmat_i.at<double>(2, 0);
                double cand_pitch = atan2(-pitch_y, sqrt(pitch_x * pitch_x + pitch_z * pitch_z));
                pnp_candidates.push_back({cand_yaw, cand_pitch});
            }
        } else {
            // 若使用了 fallback solvePnP，仍然计算单个 yaw/pitch
            cv::Mat rmat_f;
            cv::Rodrigues(rvec, rmat_f);
            double n_x = rmat_f.at<double>(0, 0);
            double n_z = rmat_f.at<double>(2, 0);
            double custom_n_x = n_z;
            double custom_n_y = -n_x;
            double pitch_x = rmat_f.at<double>(0, 0);
            double pitch_y = rmat_f.at<double>(1, 0);
            double pitch_z = rmat_f.at<double>(2, 0);
            double cand_pitch = atan2(-pitch_y, sqrt(pitch_x * pitch_x + pitch_z * pitch_z));
            pnp_candidates.push_back({atan2(custom_n_y, custom_n_x), cand_pitch});
        }

        // 根据 id 选择 pnp_yaw：id==6 选 pitch 大的，否则选 pitch 小的
        double pnp_yaw = pnp_candidates.empty() ? 0.0 : pnp_candidates[0].yaw;
        if (pnp_candidates.size() >= 2) {
            int sel = (detection.id == 6)
                ? (pnp_candidates[0].pitch >= pnp_candidates[1].pitch ? 0 : 1)
                : (pnp_candidates[0].pitch <= pnp_candidates[1].pitch ? 0 : 1);
            pnp_yaw = pnp_candidates[sel].yaw;
        }

        // 计算 pnp 重投影误差
        double pnp_reproj_err = 0.0;
        if (!rvecs.empty() && !tvecs.empty()) {
            cv::Mat projected_mat;
            cv::projectPoints(detection_points_3D, rvecs[0], tvecs[0],
                converter->getIntrinsic(), converter->getDistortion(), projected_mat);
            int npts = projected_mat.rows * projected_mat.cols;
            for (int pi = 0; pi < npts && pi < (int)detection.kpts.size(); ++pi) {
                cv::Point2d p = projected_mat.at<cv::Point2d>(pi);
                pnp_reproj_err += cv::norm(p - cv::Point2d(detection.kpts[pi]));
            }
            if (npts > 0) pnp_reproj_err /= npts;
        }

        tmp.yaw = pnp_yaw + converter->getLastYaw(); // 纠正回world坐标系的yaw
        tmp.id = detection.id;
        tmp.pnp_reproj_err = pnp_reproj_err;

        // 删除不合法的坐标
        if(!legalPos(tmp)) {
            continue;
        }
        if (!std::isfinite(tmp.yaw)) {
            continue;
        }

        res.push_back(tmp);
    }

    return res;
}