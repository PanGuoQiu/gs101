#pragma once

#include "Object.hpp"
#include "Vector.hpp"

//球体类
class Sphere : public Object
{
	public:
		//构造函数
		Sphere(const Vector3f& c, const float& r) : center(c), radius(r), radius2(r * r) {}

		//判断射线是否与球体相交
		bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t&, Vector2f&) const override
		{
			//计算一元二次方程系数a、b、c
			Vector3f L = orig - center;
			float a = dotProduct(dir, dir);
			float b = 2 * dotProduct(dir, L);
			float c = dotProduct(L, L) - radius2;
			float t0, t1;
			//解
			if (!solveQuadratic(a, b, c, t0, t1))
				return false;
			
			if (t0 < 0)
				t0 = t1;

			if (t1 < 0)
				return false;

			tnear = t0;

			return true;
		}

		//获得平面性质
		void getSurfaceProperties(const Vector3f& P, const Vector3f&, const uint32_t&, const Vector2f&, Vector3f& N, Vector2f&) const override
		{
			N = normalize(P - center);							//计算法向量N
		}

	public:
		Vector3f center;										//球心
		float radius, radius2;									//半径和半径的平方
};
