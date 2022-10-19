#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <Eigen/Eigen>
using namespace Eigen;

//三角形类
class Triangle
{
  public:
    Vector3f v[3];                                        //三角形的原始坐标，v0、v1、v2为顺时针旋转

    //每个顶点的属性值
    Vector3f color[3];                                    //每个顶点的颜色
    Vector2f tex_coords[3];                               //纹理u, v
    Vector3f normal[3];                                   //每个顶点的法向量

    //构造函数
    Triangle();

    //获得三角形每个顶点的值
    Eigen::Vector3f a() const { return v[0]; }
    Eigen::Vector3f b() const { return v[1]; }
    Eigen::Vector3f c() const { return v[2]; }

    //设置每个顶点的属性值
    void setVertex(int ind, Vector3f ver);                //设置第i个顶点的坐标
    void setNormal(int ind, Vector3f n);                  //设置第i个顶点的法向量
    void setColor(int ind, float r, float g, float b);    //设置第i个顶点的颜色
    void setTexCoord(int ind, float s, float t);          //设置纹理坐标

    std::array<Vector4f, 3> toVector4() const;            //存储三角形的三个顶点值
};

#endif //RASTERIZER_TRIANGLE_H

