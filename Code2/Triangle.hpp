#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <Eigen/Eigen>
using namespace Eigen;

//三角形类
class Triangle
{
    public:
        Vector3f v[3];                                          //三角形的原始坐标v0, v1, v2为顺时针

        //每个顶点的属性值
        Vector3f color[3];                                      //每个顶点的颜色
        Vector2f tex_coords[3];                                 //每个顶点的纹理(u, v)
        Vector3f normal[3];                                     //每个顶点的法线

        //构造函数
        Triangle();

        void setVertex(int ind, Vector3f ver);                  //设置第i个顶点的坐标
        void setNormal(int ind, Vector3f n);                    //设置第i个顶点的法线
        void setColor(int ind, float r, float g, float b);      //设置第i个顶点的颜色

        Vector3f getColor() const { return color[0] * 255; }    //每个三角形只有一个颜色
        void setTexCoord(int ind, float s, float t);            //设置第i个顶点的纹理坐标

        //把三角形的三个顶点转换为4维向量，并存储在array数组中
        std::array<Vector4f, 3> toVector4() const;
};

#endif //RASTERIZER_TRIANGLE_H
