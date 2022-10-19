#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <Eigen/Eigen>
#include "Texture.hpp"
using namespace Eigen;

//三角形类
class Triangle
{
    public:
        Vector4f v[3];                                              //三角形的原始坐标，v0、v1、v2为逆时针顺序

        //每个顶点的属性
        Vector3f color[3];                                          //每个顶点的颜色
        Vector2f tex_coords[3];                                     //纹理(u, v)
        Vector3f normal[3];                                         //每个顶点的法线

        Texture* tex = nullptr;                                     //纹理数据指针

        //构造函数
        Triangle();

        //获得三角形的三个顶点坐标
        Eigen::Vector4f a() const { return v[0]; }
        Eigen::Vector4f b() const { return v[1]; }
        Eigen::Vector4f c() const { return v[2]; }

        //设置顶点的属性
        void setVertex(int ind, Vector4f ver);                      //设置第ind个顶点的坐标
        void setNormal(int ind, Vector3f n);                        //设置第ind个顶点的法线
        void setColor(int ind, float r, float g, float b);          //设置第ind个顶点的颜色

        void setNormals(const std::array<Vector3f, 3>& normals);    //array顶点的法线转换为三角形normal的法线
        void setColors(const std::array<Vector3f, 3>& colors);      //array顶点的颜色转换为三角形color的颜色
        void setTexCoord(int ind, Vector2f uv);                     //设置第ind个顶点的纹理坐标

        //把三角形的3维顶点转换为4维的顶点
        std::array<Vector4f, 3> toVector4() const;
};

#endif //RASTERIZER_TRIANGLE_H
