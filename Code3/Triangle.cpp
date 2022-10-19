#include "Triangle.hpp"
#include <algorithm>
#include <array>

//构造函数
Triangle::Triangle()
{
    //初始化位置
    v[0] << 0, 0, 0, 1;
    v[1] << 0, 0, 0, 1;
    v[2] << 0, 0, 0, 1;

    //初始化颜色
    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    //初始化纹理坐标
    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

//设置第ind个顶点的坐标
void Triangle::setVertex(int ind, Vector4f ver)
{
    v[ind] = ver;
}

//设置第ind个顶点的法线
void Triangle::setNormal(int ind, Vector3f n)
{
    normal[ind] = n;
}

//设置第ind个顶点的颜色
void Triangle::setColor(int ind, float r, float g, float b)
{
    //判断颜色值是否有效
    if ((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) || (b < 0.0) || (b > 255.))
    {
        fprintf(stderr, "错误无效的颜色值");
        fflush(stderr);
        exit(-1);
    }
    
    color[ind] = Vector3f((float)r/255., (float)g/255., (float)b/255.);
    return;
}

//设置第ind个顶点的纹理坐标
void Triangle::setTexCoord(int ind, Vector2f uv)
{
    tex_coords[ind] = uv;
}

//3维转换4维
std::array<Vector4f, 3> Triangle::toVector4() const
{
    std::array<Vector4f, 3> res;
    std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec) { return Vector4f(vec.x(), vec.y(), vec.z(), 1.0f); });
    return res;
}

//array法线数组转换为三角形的法线数组
void Triangle::setNormals(const std::array<Vector3f, 3>& normals)
{
    normal[0] = normals[0];
    normal[1] = normals[1];
    normal[2] = normals[2];
}

//array颜色数组转换为三角形的颜色数组
void Triangle::setColors(const std::array<Vector3f, 3>& colors)
{
    auto first_color = colors[0];
    setColor(0, colors[0][0], colors[0][1], colors[0][2]);
    setColor(1, colors[1][0], colors[1][1], colors[1][2]);
    setColor(2, colors[2][0], colors[2][1], colors[2][2]);
}
