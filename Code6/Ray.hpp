#ifndef RAYTRACING_RAY_H
#define RAYTRACING_RAY_H

#include "Vector.hpp"

//射线结构体
struct Ray
{
    //Destination = origin + t * direction(终点 = 原点 + 系数t * 方向)
    Vector3f origin;                                            //原点
    Vector3f direction, direction_inv;                          //方向，1/方向：因为乘法运算比除法运算快
    double t;                                                   //移动时间
    double t_min, t_max;                                        //t的最小值和最大值

    //构造函数
    Ray(const Vector3f& ori, const Vector3f& dir, const double _t = 0.0) : origin(ori), direction(dir), t(_t) 
    {
        direction_inv = Vector3f(1.0 / direction.x, 1.0 / direction.y, 1.0 / direction.z);
        t_min = 0.0;
        t_max = std::numeric_limits<double>::max();
    }

    //重载()运算符：类函数
    Vector3f operator()(double t) const
    {
        return origin + direction * t;                          //返回目标点向量
    }

    //重载<<插入运算符(输出射线参数)
    friend std::ostream& operator<<(std::ostream& os, const Ray& r)
    {
        os<<"[origin:="<<r.origin<<", direction="<<r.direction<<", time="<<r.t<<"]\n";
        return os;
    }
};

#endif //RAYTRACING_RAY_H
