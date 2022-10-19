#pragma once
#ifndef RAYTRACING_VECTOR_H
#define RAYTRACING_VECTOR_H

#include <iostream>
#include <cmath>
#include <algorithm>

//3维向量类
class Vector3f
{
    public:
        //各种构造函数
        Vector3f() : x(0), y(0), z(0) {}
        Vector3f(float xx) : x(xx), y(xx), z(xx) {}
        Vector3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}

        //重载运算符，数量与向量的运算
        Vector3f operator * (const float& r) const { return Vector3f(x * r, y * r, z * r); }
        Vector3f operator / (const float& r) const { return Vector3f(x / r, y / r, z / r); }

        //求模
        float norm() { return std::sqrt(x * x + y * y + z * z); }
        //单位化
        Vector3f normalized()
        {
            float n = std::sqrt(x * x + y * y + z * z);
            return Vector3f(x / n, y / n, z / n);
        }

        //重载运算符，向量与向量运算
        Vector3f operator * (const Vector3f& v) const { return Vector3f(x * v.x, y * v.y, z * v.z); }
        Vector3f operator - (const Vector3f& v) const { return Vector3f(x - v.x, y - v.y, z - v.z); }
        Vector3f operator + (const Vector3f& v) const { return Vector3f(x + v.x, y + v.y, z + v.z); }
        Vector3f operator - () const { return Vector3f(-x, -y, -z); }
        Vector3f& operator += (const Vector3f& v) { x += v.x, y += v.y, z += v.z; return *this; }

        //左乘
        friend Vector3f operator * (const float& r, const Vector3f& v)
        {
            return Vector3f(v.x * r, v.y * r, v.z * r);
        }

        //重载<<插入运算符
        friend std::ostream& operator << (std::ostream& os, const Vector3f& v)
        {
            return os << v.x << ", " << v.y << ", " << v.z;
        }

        //重载[]运算符
        double operator[](int index) const;
        double& operator[](int index);

        //取两个向量的各个分量的最小值
        static Vector3f Min(const Vector3f& p1, const Vector3f& p2)
        {
            return Vector3f(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z));
        }

        //取两个向量的各个分量的最大值
        static Vector3f Max(const Vector3f& p1, const Vector3f& p2)
        {
            return Vector3f(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z));
        }

    public:
        float x, y, z;
};

//重载[]运算符
inline double Vector3f::operator[](int index) const
{
    return (&x)[index];
}

//二维向量类
class Vector2f
{
    public:
        //各种构造函数
        Vector2f() : x(0), y(0) {}
        Vector2f(float xx) : x(xx), y(xx) {}
        Vector2f(float xx, float yy) : x(xx), y(yy) {}

        //重载运算符
        Vector2f operator * (const float& r) const { return Vector2f(x * r, y * r); }
        Vector2f operator + (const Vector2f& v) const { return Vector2f(x + v.x, y + v.y); }

    public:
        float x, y;
};

//线性插值
inline Vector3f lerp(const Vector3f& a, const Vector3f& b, const float& t)
{
    return a * (1 - t) + b * t;
}

//单位化
inline Vector3f normalize(const Vector3f& v)
{
    float mag2 = v.x * v.x + v.y * v.y + v.z * v.z;
    if (mag2 > 0)
    {
        float invMag = 1 / sqrtf(mag2);
        return Vector3f(v.x * invMag, v.y * invMag, v.z * invMag);
    }

    return v;
}

//点乘
inline float dotProduct(const Vector3f& a, const Vector3f& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

//向量的平方
inline float squaredNorm(const Vector3f& a)
{
    return dotProduct(a, a);
}

//向量的模
inline float norm(const Vector3f& a)
{
    return sqrt(dotProduct(a, a));
}

//叉乘
inline Vector3f crossProduct(const Vector3f& a, const Vector3f& b)
{
    return Vector3f(a.y * b.z - a.z * b.y,  a.z * b.x - a.x * b.z,  a.x * b.y - a.y * b.x);
}

#endif //RAYTRACING_VECTOR_H
