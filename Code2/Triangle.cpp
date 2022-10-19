#include "Triangle.hpp"
#include <algorithm>
#include <array>

//构造函数
Triangle::Triangle()
{
    v[0] << 0, 0, 0;
    v[1] << 0, 0, 0;
    v[2] << 0, 0, 0;

    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

//设置第i个顶点的位置
void Triangle::setVertex(int ind, Vector3f ver)
{
    v[ind] = ver;
}

//设置第i个顶点的法线
void Triangle::setNormal(int ind, Vector3f n)
{
    normal[ind] = n;
}

//设置第i个顶点的颜色
void Triangle::setColor(int ind, float r, float g, float b)
{
    //判断颜色是否有效
    if ((r < 0.0) || (r > 255.) || (g < 0) || (g > 255.) || (b < 0) || (b > 255.))
    {
        fprintf(stderr, "错误！无效颜色值");
        fflush(stderr);
        exit(-1);
    }

    //设置有效颜色值
    color[ind] = Vector3f((float)r/255., (float)g/255., (float)b/255.);
    return;
}

//设置第i个顶点的纹理坐标
void Triangle::setTexCoord(int ind, float s, float t)
{
    tex_coords[ind] = Vector2f(s, t);
}

//把三角形的三个顶点转换为4维向量，并存储在array数组中
std::array<Vector4f, 3> Triangle::toVector4() const
{
    std::array<Eigen::Vector4f, 3> res;
    std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec) { return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.f); });

    return res;
}
