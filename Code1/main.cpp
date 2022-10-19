#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

//计算视图矩阵
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    //初始化一个视图单位矩阵
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    //基轴固定，位置不同的摄像机
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],  0, 1, 0, -eye_pos[1],  0, 0, 1, -eye_pos[2],  0, 0, 0, 1;

    //计算视图矩阵
    view = translate * view;

    return view;
}

//计算模型矩阵
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    //初始化一个模型单位矩阵
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    //待办：执行这个函数
    //创建一个模型矩阵使三角形绕Z轴旋转
    //然后，返回模型矩阵
    float _cos = cos(rotation_angle);
    float _sin = sin(rotation_angle);
    
    //设置模型矩阵
    model(0, 0) = _cos;
    model(0, 1) = -_sin;

    model(1, 0) = _sin;
    model(1, 1) = _cos;

    return model;
}


//计算投影矩阵，作业填写投影矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    //初始化一个投影单位矩阵
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    //计算矩阵元素值
    float _tan = tan(eye_fov / 2);
    projection << 1 / (aspect_ratio * _tan), 0, 0, 0, 
                  0, -1 / _tan, 0, 0, 
                  0, 0, (zNear + zFar) / (zNear - zFar), 2 * zNear * zFar / (zFar - zNear), 
                  0, 0, 1, 0;

    return projection;
}

//主函数
int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]);                 //-r为默认值
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    //光栅化对象
    rst::rasterizer r(700, 700);

    //设置摄像机位置
    Eigen::Vector3f eye_pos = {0, 0, 5};

    //设置三角形的三个顶点
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    //设置三角形三个顶点的索引
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    //插入光栅化对象中
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    //按键和帧数
    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        //清空帧缓存和深度缓存
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //设置MVP矩阵
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        //绘制，并生成图片
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    //按键处理
    while (key != 27)
    {
        //重新清除缓存
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //重新设置MVP
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        //重新绘制
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        //生成图片
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout<<"frame count: "<<frame_count++<<'\n';

        //旋转按键a和d
        if (key == 'a') 
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}