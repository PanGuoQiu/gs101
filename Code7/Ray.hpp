#ifndef RAYTRACING_RAY_H
#define RAYTRACING_RAY_H

#include "Vector.hpp"

//射线类
struct Ray
{
    //Destination = origin + t * direction (终点 = 原点 + 时间 * 方向)
    Vector3f origin;                                    //原点
    Vector3f direction, direction_inv;                  //方向，倒置的方向
    double t;                                           //传输的时间
    double t_min, t_max;                                //时间

    //构造函数
    Ray(const Vector3f& ori, const Vector3f& dir, const double _t = 0.0) : origin(ori), direction(dir), t(_t) 
    {
        direction_inv = Vector3f(1. / direction.x, 1. / direction.y, 1. / direction.z);
        t_min = 0.0;
        t_max = std::numeric_limits<double>::max();
    }

    //重载()运算符
    Vector3f operator()(double t) const
    {
        return origin + direction * t;
    }

    //重载<<插入运算符 (即输出Ray类型数据)
    friend std::ostream& operator<<(std::ostream& os, const Ray& r)
    {
        os<<"[origin:="<<r.origin<<", direction="<<r.direction<<", time="<<r.t<<"]\n";
        return os;
    }
};

#endif //RAYTRACING_RAY_H
