#pragma once

#include <cstdio>
#include <opencv2/opencv.hpp>

#include <unit/unit.hpp>

namespace utils::img {

// 自适应分辨率的图片显示, 用法和OpenCV一致
void imshow(const cv::String &winname, const cv::Mat &img);

}

namespace utils::math {

// 限制弧度到-pi到pi
unit::literals::radian limAngle(unit::literals::radian rad);

// 限制弧度到 (-pi, pi]，double 精度版本
// 与 limAngle(radian) 行为一致，但避免 unit::literals::radian 内部 float 截断
double limAngle(double rad);

/**
 * 将一个角度吸引到另一个角度
 * 例如: 如果 a 和 b 相差几乎是 PI / 2 的整数倍, 可以用 b = attractTo(b, a, PI / 2) 将 b 吸引到 a 附近
 * 例如：x = attractTo(x, 0, PI * 2) 和 x = limAngle(x) 效果相同
 * 
 * @param src 起始角度
 * @param dst 目标角度
 * @param step 步长
 * @param nstep 返回经过多少步 (ret = src + step * nstep)
*/
double attractTo(double src, double dst, double step, int *nstep = NULL);

}

namespace utils::seek{
    // 数组中找到最小元素
    template<typename _Tp, typename _Compare> inline
    _Tp minElement(std::vector<_Tp> &list, _Compare comp) {
        if (list.empty()) {
            printf("[ERROR] empty list in minElement()\n");
            return _Tp();
        }

        _Tp &ret = std::move(list.at(0));
        for (_Tp &tmp: list) {
            if (comp(tmp, ret)) { // tmp < ret
                ret = tmp;
            }
        }
        return ret;
    }

    // 数组中找到最大元素
    template<typename _Tp, typename _Compare> inline
    _Tp maxElement(std::vector<_Tp> &list, _Compare comp) {
        if (list.empty()) {
            printf("[ERROR] empty list in maxElement()\n");
            return _Tp();
        }

        _Tp &ret = std::move(list.at(0));
        for (_Tp &tmp: list) {
            if (!comp(tmp, ret)) { // tmp >= ret
                ret = tmp;
            }
        }
        return ret;
    }

    template<typename _Tp, typename _GetValue, typename _Comp> static inline
    double __getElementByValue(std::vector<_Tp> &list, _Tp &elem, int &ret_idx,
                            _GetValue getv, _Comp comp) {
        ret_idx = 0;
        double dst_value = (double)getv(list.at(0));

        for (int i = 1; i < list.size(); i++) {
            double curr_value = (double)getv(list.at(i));
            if (comp(curr_value, dst_value)) {
                dst_value = curr_value;
                ret_idx = i;
            }
        }
        elem = list.at(ret_idx);
        return dst_value;
    }

    // 通过获得一个值找到最小元素索引
    template<typename _Tp, typename _GetValue> inline
    int minIndexByValue(std::vector<_Tp> &list, _GetValue getv) {
        if (list.empty()) {
            printf("[ERROR] empty list in minIndexByValue()\n");
            return -1;
        }

        _Tp ret;
        int ret_idx;
        __getElementByValue(list, ret, ret_idx, getv,
            [](double &a, double &b) { return a < b; }
        );
        return ret_idx;
    }

    // 通过获得一个值找到最大元素索引
    template<typename _Tp, typename _GetValue> inline
    int maxIndexByValue(std::vector<_Tp> &list, _GetValue getv) {
        if (list.empty()) {
            printf("[ERROR] empty list in maxIndexByValue()\n");
            return _Tp();
        }

        _Tp ret;
        int ret_idx;
        __getElementByValue(list, ret, ret_idx, getv,
            [](double &a, double &b) { return a > b; }
        );
        return ret_idx;
    }

    // 通过获得一个值找到最小元素
    template<typename _Tp, typename _GetValue> inline
    _Tp minElementByValue(std::vector<_Tp> &list, _GetValue getv) {
        if (list.empty()) {
            printf("[ERROR] empty list in minElementByValue()\n");
            return _Tp();
        }

        _Tp ret;
        int ret_idx;
        __getElementByValue(list, ret, ret_idx, getv,
            [](double &a, double &b) { return a < b; }
        );
        return ret;
    }
    // 通过获得一个值找到最小元素, 返回最小值
    template<typename _Tp, typename _GetValue> inline
    double minElementByValue(std::vector<_Tp> &list, _GetValue getv, _Tp &ret) {
        if (list.empty()) {
            printf("[ERROR] empty list in minElementByValue()\n");
            return 0;
        }

        int ret_idx;
        return __getElementByValue(list, ret, ret_idx, getv,
            [](double &a, double &b) { return a < b; }
        );
    }

    // 通过获得一个值找到最大元素
    template<typename _Tp, typename _GetValue> inline
    _Tp maxElementByValue(std::vector<_Tp> &list, _GetValue getv) {
        if (list.empty()) {
            printf("[ERROR] empty list in maxElementByValue()\n");
            return _Tp();
        }

        _Tp ret;
        int ret_idx;
        __getElementByValue(list, ret, ret_idx, getv,
            [](double &a, double &b) { return a > b; }
        );
        return ret;
    }

    // 通过获得一个值找到最大元素, 返回最大值
    template<typename _Tp, typename _GetValue> inline
    double maxElementByValue(std::vector<_Tp> &list, _GetValue getv, _Tp &ret) {
        if (list.empty()) {
            printf("[ERROR] empty list in maxElementByValue()\n");
            return 0;
        }

        int ret_idx;
        return __getElementByValue(list, ret, ret_idx, getv,
            [](double &a, double &b) { return a > b; }
        );
    }
}