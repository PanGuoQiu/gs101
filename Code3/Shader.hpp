#ifndef RASTERIZER_SHADER_H
#define RASTERIZER_SHADER_H

#include <Eigen/Eigen>
#include "Texture.hpp"

//有效载入片段着色结构体
struct fragment_shader_payload
{
    //构造函数
    fragment_shader_payload()
    {
        texture = nullptr;
    }

    //重载构造函数
    fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor, const Eigen::Vector2f& tc, Texture* tex) :
                            color(col), normal(nor), tex_coords(tc), texture(tex) 
    {
    }

    //片段着色需要的自定变量
    Eigen::Vector3f view_pos;                                   //像素点的位置
    Eigen::Vector3f color;                                      //颜色
    Eigen::Vector3f normal;                                     //法线
    Eigen::Vector2f tex_coords;                                 //纹理坐标
    Texture* texture;                                           //纹理数据
};

//有效载入顶点着色结构体
struct vertex_shader_payload
{
    Eigen::Vector3f position;                                   //顶点位置
};

#endif //RASTERIZER_SHADER_H
