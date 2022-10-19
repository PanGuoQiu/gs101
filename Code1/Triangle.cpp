#include "Triangle.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>

//构造函数
Triangle::Triangle()
{
    //初始化三角形每个顶点的属性值
    //位置
    v[0] << 0, 0, 0;
    v[1] << 0, 0, 0;
    v[2] << 0, 0, 0;

    //颜色
    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    //纹理
    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

//设置索引顶点位置
void Triangle::setVertex(int ind, Eigen::Vector3f ver)
{
    v[ind] = ver;
}

//设置索引顶点法向量
void Triangle::setNormal(int ind, Vector3f n)
{
    normal[ind] = n;
}

//设置顶点颜色
void Triangle::setColor(int ind, float r, float g, float b)
{
    //如果颜色值错误则抛出异常
    if ((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) || (b < 0.0) || (b > 255.))
    {
        throw std::runtime_error("颜色值错误");
    }

    color[ind] = Vector3f((float)r / 255., (float)g / 255., (float)b / 255.);
    return;
}

//设置纹理
void Triangle::setTexCoord(int ind, float s, float t)
{
    tex_coords[ind] = Vector2f(s, t);
}

//存储三角形的三个顶点
std::array<Vector4f, 3> Triangle::toVector4() const
{
    std::array<Vector4f, 3> res;
    std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec){
        return Vector4f(vec.x(), vec.y(), vec.z(), 1.f);
    });

    return res;
}
