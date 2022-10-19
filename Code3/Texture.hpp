#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H

#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

//纹理类
class Texture
{
    private:
        cv::Mat image_data;                                         //纹理数据
    
    public:
        //构造函数
        Texture(const std::string& name)
        {
            image_data = cv::imread(name);                          //读取图片数据并保存
            cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);//设置颜色格式
            width = image_data.cols;                                //纹理宽度
            height = image_data.rows;                               //纹理高度
        }

        int width, height;                                          //纹理宽度和高度

        //获得uv纹理坐标
        Eigen::Vector3f getColor(float u, float v)
        {
            auto u_img = u * width;                                 //纹理对应的图片坐标宽度
            auto v_img = (1 - v) * height;                          //纹理对应的图片坐标高度
            auto color = image_data.at<cv::Vec3b>(v_img, u_img);    //(u, v)处理的颜色
            
            return Eigen::Vector3f(color[0], color[1], color[2]);   //返回颜色
        }
};

#endif //RASTERIZER_TEXTURE_H
