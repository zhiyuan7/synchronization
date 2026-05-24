#include "utils/utils.hpp"
#include <cmath>
#include <stdexcept>


// 获取屏幕分辨率
static inline void _initResolution(int &w, int &h) {
    char command[] = "xrandr | grep '*'";
    FILE *fpipe;

    // 防止报错以后程序停止运行，毕竟这只是debug功能
    try {
        fpipe = (FILE *)popen(command, "r");
    } catch (...) {
        return;
    }
    
    int _w, _h;
    // 解析xrandr输出，格式通常是 "1920x1080+0+0"
    if (fscanf(fpipe, "%dx%d", &_w, &_h) != 2) {
        pclose(fpipe);
        return;
    }
    pclose(fpipe);

    // 应该还没有人用 8K 屏调车吧
    if (_w < 0 || _w > 8192) {
        return;
    }
    if (_h < 0 || _h > 8192) {
        return;
    }
    
    w = _w;
    h = _h;
}

void utils::img::imshow(const cv::String &winname, const cv::Mat &img) {
    static bool resolution_init = false;
    static int w = 1440, h = 1024;
    if (!resolution_init) {
        _initResolution(w, h);
        resolution_init = true;
    }

    // 缩小的倍率
    double scale = std::min((double)w / img.cols, (double)h / img.rows);
    scale *= 0.9; // 图片不超过全屏的 0.9
    // scale = std::min(1., scale); // 图片太小不放大

    cv::Mat show_im = img.clone();
    cv::resize(show_im, show_im, cv::Size(img.cols * scale, img.rows * scale));
    cv::imshow(winname, show_im);
}

unit::literals::radian utils::math::limAngle(unit::literals::radian rad) {
    return unit::literals::radian(std::remainder(rad.value, 2.0 * unit::literals::UNIT_PI));
}

double utils::math::limAngle(double rad) {
    return std::remainder(rad, 2.0 * unit::literals::UNIT_PI);
}

double utils::math::attractTo(double src, double dst, double step, int *nstep) {
    if (std::fabs(step) < 1e-6) {
        throw std::invalid_argument("[utils::math::attractTo] step cannot be 0");
    }

    // handle cases that step < 0
    int sign = (step > 0) ? 1 : -1;
    step = std::fabs(step);

    double diff = src - dst;
    // 先粗略缩小范围
    int step_n = (int)(diff / step);
    diff -= step_n * step;

    while (diff > step / 2.0) diff -= step;
    while (diff < -step / 2.0) diff += step;
    
    if (nstep != NULL) {
        *nstep = (int)round((dst + diff - src) / step);
        *nstep *= sign;
    }
    return dst + diff;
}