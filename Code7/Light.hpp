#pragma once

#include "Vector.hpp"

//光源类
class Light
{
	public:
		//构造函数和虚析构函数
		Light(const Vector3f& p, const Vector3f& i) : position(p), intensity(i) {}
		virtual ~Light() = default;

	public:
		Vector3f position;						//位置
		Vector3f intensity;						//强度
};
