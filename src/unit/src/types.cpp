#include "unit/types.hpp"


// -------------------------------------
// -----------------灯条-----------------
// -------------------------------------

// 获取直线参数  
void getLinePara(float x1, float y1, float x2, float y2, LinePara &LP)
{
	double m = 0;

	// 计算分子  
	m = x2 - x1;

	if (std::fabs(m) < 0.0001) {
		LP.k = 10000.0;
		LP.b = y1 - LP.k * x1;
	} else {
		LP.k = (y2 - y1) / (x2 - x1);
		LP.b = y1 - LP.k * x1;
	}
}

bool getCrossPoint(
    const cv::Point2f &p1, const cv::Point2f &p2,
    const cv::Point2f &p3, const cv::Point2f &p4, 
    cv::Point2f &cross_point
) {
    LinePara LP1, LP2;
    getLinePara(p1.x, p1.y, p2.x, p2.y, LP1);
    getLinePara(p3.x, p3.y, p4.x, p4.y, LP2);

    // 判断两条直线是否平行（斜率相等）
    if (std::fabs(LP1.k - LP2.k) < 0.0001) {
        return false;
    } else {
        cross_point.x = (LP2.b - LP1.b) / (LP1.k - LP2.k);
        cross_point.y = LP1.k * cross_point.x + LP1.b;
        return true;
    }
}

Light::Light(cv::Point2f _top, cv::Point2f _bottom):
    top(_top),
    bottom(_bottom),
    width(0),
    color(TeamColor::RED),
    center((_top + _bottom) / 2.0),
    length(cv::norm(_top - _bottom))
{
}

Light::Light(const std::vector<cv::Point> &cnt)
: contour(cnt) {
    cv::RotatedRect rect = cv::minAreaRect(cnt);

    // 根据轮廓点数选择端点计算方法
    std::vector<cv::Point2f> top_bottom = (cnt.size() < 5) 
        ? getLightPointsByMinAreaRect(rect)
        : getLightPointsByEllipse(cv::fitEllipse(cnt), rect);

    top = top_bottom.at(0);
    bottom = top_bottom.at(1);
    center = (top + bottom) / 2.0;
    bounding_rect = cv::boundingRect(cnt);
    length = cv::norm(bottom - top);

    // 计算宽度
    cv::Point2f corners[4];
    rect.points(corners);
    std::sort(
        corners, corners + 4,
        [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; }
    );
    width = (cv::norm(corners[0] - corners[1]) + cv::norm(corners[2] - corners[3])) / 2.0;
}

bool Light::operator==(const Light &other) const {
    constexpr float eps = 1e-3f;
    return cv::norm(top - other.top) < eps && cv::norm(bottom - other.bottom) < eps;
}

std::vector<cv::Point2f>
Light::getLightPointsByMinAreaRect(cv::RotatedRect rotate_rect) {
    cv::Point2f corners[4];
    // 旋转矩形4点
    rotate_rect.points(corners);
    std::sort(
        corners, corners + 4,
        [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; }
    );
    return {
        (corners[0] + corners[1]) / 2.0,
        (corners[2] + corners[3]) / 2.0
    };
}

std::vector<cv::Point2f>
Light::getLightPointsByEllipse(cv::RotatedRect ellipse, cv::RotatedRect rotate_rect) {
    cv::Point2f corners[4];

    // 椭圆两个端点
    ellipse.points(corners);
    std::sort(
        corners, corners + 4,
        [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; }
    );
    cv:: Point2f elli_top = (corners[0] + corners[1]) / 2.0;
    cv:: Point2f elli_bottom = (corners[2] + corners[3]) / 2.0;

    // 旋转矩形4点
    rotate_rect.points(corners);
    std::sort(
        corners, corners + 4,
        [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; }
    );

    cv::Point2f res_top;
    cv::Point2f res_bottom;
    // 椭圆长轴与旋转矩形的2个交点
    getCrossPoint(corners[0], corners[1], elli_top, elli_bottom, res_top); 
    getCrossPoint(corners[2], corners[3], elli_top, elli_bottom, res_bottom);

    return {res_top, res_bottom};
}
