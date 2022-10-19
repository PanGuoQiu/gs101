#pragma once

#include "Vector.hpp"
#include "global.hpp"

//对象类
class Object
{
	public:
		//构造函数和虚析构函数
		Object() : materialType(DIFFUSE_AND_GLOSSY), ior(1.3), Kd(0.8), Ks(0.2), diffuseColor(0.2), specularExponent(25) {}
		virtual ~Object() = default;

		//对象是否与射线相交
		virtual bool intersect(const Vector3f&, const Vector3f&, float&, uint32_t&, Vector2f&) const = 0;

		//获得平面的性质
		virtual void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t&, const Vector2f&, Vector3f&, Vector2f&) const = 0;

		//重新计算散射颜色(默认)
		virtual Vector3f evalDiffuseColor(const Vector2f&) const
		{
			return diffuseColor;
		}

	public:
		//对象性质
		MaterialType materialType;								//材质类型
		float ior;												//材质的折射率
		float Kd, Ks;											//散射系数和镜面系数
		Vector3f diffuseColor;									//散射颜色
		float specularExponent;									//镜面指数
};
