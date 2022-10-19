#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

//获得视图矩阵
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;
    return view;
}

//获得模型矩阵
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    return model;
}

//获得投影矩阵
//计算投影矩阵，作业填写投影矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    //初始化一个投影单位矩阵
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    //计算矩阵元素值
    float _tan = tan(eye_fov / 2);
    projection << -1 / (aspect_ratio * _tan), 0, 0, 0, 
                  0, -1 / _tan, 0, 0, 
                  0, 0, (zNear + zFar) / (zNear - zFar), 2 * zNear * zFar / (zFar - zNear), 
                  0, 0, 1, 0;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    //光栅化的窗口大小
    rst::rasterizer r(700, 700);

    //设置摄像机位置
    Eigen::Vector3f eye_pos = {0, 0, 5};

    //设置顶点位置
    std::vector<Eigen::Vector3f> pos
    {
        {2, 0, -2}, 
        {0, 2, -2}, 
        {-2, 0, -2}, 
        {3.5, -1, -5}, 
        {2.5, 1.5, -5}, 
        {-1, 0.5, -5}
    };

    //顶点索引
    std::vector<Eigen::Vector3i> ind
    {
        {0, 1, 2}, 
        {3, 4, 5}
    };

    //顶点颜色
    std::vector<Eigen::Vector3f> col
    {
        {217.0, 238.0, 185.0}, 
        {217.0, 238.0, 185.0}, 
        {217.0, 238.0, 185.0}, 
        {185.0, 217.0, 238.0},
        {185.0, 217.0, 238.0},
        {185.0, 217.0, 238.0}
    };

    //载入三角形顶点属性到光栅化类中
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(col);

    //帧缓存和按键
    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        //清除颜色缓冲区域和深度缓冲区域
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //设置MVP矩阵
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        //绘制三角形、并保存为图片
        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    //按键处理
    while (key != 27)
    {
        //清除颜色缓冲区域和深度缓冲区域
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //设置MVP矩阵
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        //绘制三角形并显示
        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        //输出帧数
        std::cout<<"frame count: "<<frame_count++<<'\n';
    }

    return 0;
}
