#pragma once

#include "Vector.hpp"
#include "Light.hpp"
#include "global.hpp"

//面积光类
class AreaLight : public Light
{
    public:
        //构造函数 : Light构造函数参数：位置和强度
        AreaLight(const Vector3f& p, const Vector3f& i) : Light(p, i)
        {
            normal = Vector3f(0, -1, 0);
            u = Vector3f(1, 0, 0);
            v = Vector3f(0, 0, 1);
            length = 100;
        }

        //获得样本点(在面积光上随机取一点)的位置
        Vector3f SamplePoint() const
        {
            auto random_u = get_random_float();
            auto random_v = get_random_float();

            return position + random_u * u + random_v * v;
        }

    public:
        float length;                           //面积光的边长度
        Vector3f normal;                        //法向量
        Vector3f u;                             //面积光u坐标轴
        Vector3f v;                             //面积光v坐标轴
};
