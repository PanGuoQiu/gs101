#pragma once

#include <cmath>
#include <iostream>

//3维向量类
class Vector3f
{
    public:
        //各种类型的构造函数
        Vector3f() : x(0), y(0), z(0) {}
        Vector3f(float xx) : x(xx), y(xx), z(xx) {}
        Vector3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}

        //重载运算符
        //重载乘号运算符 (缩放)
        Vector3f operator*(const float& r) const
        {
            return Vector3f(x * r, y * r, z * r);
        }

        //重载除号运算符
        Vector3f operator/(const float& r) const
        {
            return Vector3f(x / r, y / r, z / r);
        }

        //重载向量相乘
        Vector3f operator*(const Vector3f& v) const
        {
            return Vector3f(x * v.x, y * v.y, z * v.z);
        }

        //重载建好运算符
        Vector3f operator-(const Vector3f& v) const
        {
            return Vector3f(x - v.x, y - v.y, z - v.z);
        }

        //重载加号运算符
        Vector3f operator+(const Vector3f& v) const
        {
            return Vector3f(x + v.x, y + v.y, z + v.z);
        }

        //重载取反运算符
        Vector3f operator-() const
        {
            return Vector3f(-x, -y, -z);
        }

        //重载加等运算符
        Vector3f& operator+=(const Vector3f& v)
        {
            x += v.x, y += v.y, z += v.z;
            return *this;
        }
		
		//友元函数：重载乘号运算符(左乘)
		friend Vector3f operator*(const float& r, const Vector3f& v)
		{
			return Vector3f(v.x * r, v.y * r, v.z * r);
		}

		//友元函数：重载插入运算符(赋值)
		friend std::ostream& operator<<(std::ostream& os, const Vector3f& v)
		{
			return os << v.x << ", " << v.y << ", " << v.z;
		}

    public:
        float x, y, z;
};

//2维向量类
class Vector2f
{
	public:
		//各种构造函数
		Vector2f() : x(0), y(0) {}
		Vector2f(float xx) : x(xx), y(xx) {}
		Vector2f(float xx, float yy) : x(xx), y(yy) {}

		//重载运算符
		//重载乘号运算符(缩放)
		Vector2f operator*(const float& r) const
		{
			return Vector2f(x * r, y * r);
		}

		//重载加号运算符(向量相加)
		Vector2f operator+(const Vector2f& v) const
		{
			return Vector2f(x + v.x, y + v.y);
		}

	public:
		float x, y;
};

//线性插值
inline Vector3f lerp(const Vector3f& a, const Vector3f& b, const float& t)
{
	return a * (1 - t) + b * t;
}

//单位化向量
inline Vector3f normalize(const Vector3f& v)
{
	float mag2 = v.x * v.x + v.y * v.y + v.z * v.z;
	if (mag2 > 0)
	{
		float invMag = 1 / sqrt(mag2);
		return Vector3f(v.x * invMag, v.y * invMag, v.z * invMag);
	}

	return v;
}

//点乘
inline float dotProduct(const Vector3f& a, const Vector3f& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

//叉乘
inline Vector3f crossProduct(const Vector3f& a, const Vector3f& b)
{
	return Vector3f(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
