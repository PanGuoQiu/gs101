#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

//控制点数组
std::vector<cv::Point2f> control_points;

//鼠标处理器
void mouse_handler(int event, int x, int y, int flags, void* userdata)
{
    //事件是否为左按钮和在控制点内
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
    {
        //设置控制点
        std::cout<<"Left button of the mouse is clicked - position ("<<x<<", "<<y<<")"<<'\n';
        control_points.emplace_back(x, y);
    }
}

//天真的贝塞尔
void naive_bezier(const std::vector<cv::Point2f>& points, cv::Mat& window)
{
    auto& p_0 = points[0];
    auto& p_1 = points[1];
    auto& p_2 = points[2];
    auto& p_3 = points[3];

    //遍历贝塞尔曲线的每个取样点，并设置颜色
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 + 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

//递归贝塞尔曲线
cv::Point2f recursize_bezier(const std::vector<cv::Point2f>& control_points, float t)
{
    //待办事项：算法
    if (control_points.size() == 2)
        return cv::Point2f(control_points[0] * (1 - t) + control_points[1] * t);

    std::vector<cv::Point2f> new_points;
    for (size_t i = 0; i < control_points.size() - 1; ++i)
    {
        new_points.push_back(control_points[i] * (1 - t) + control_points[i + 1] * t);
    }

    return recursize_bezier(new_points, t);
}

//贝塞尔曲线
void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
{
    //待办事项：遍历所有t=0到t=1的微小步幅，并调用严格的递归贝塞尔算法
    int num = 1000;
    float t = 0, step = 1.0 / 1000;
    for (int i = 1; i != num - 1; ++i)
    {
        t = i * step;
        cv::circle(window, recursize_bezier(control_points, t), 1.5, {255, 0, 0}, 3);
    }
}

//主函数
int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27)
    {
        for (auto& point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4)
        {
            naive_bezier(control_points, window);
            //bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
