#pragma once

#include <cmath>
#include <iostream>
#include <random>

#define M_PI 3.14159265358979323846

constexpr float kInfinity = std::numeric_limits<float>::max();

//v取值范围是否在lo和hi之间
inline float clamp(const float& lo, const float& hi, const float& v)
{
	return std::max(lo, std::min(hi, v));
}

//解一元二次方程
inline bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1)
{
	float discr = b * b - 4 * a * c;
	if (discr < 0)
		return false;
	else if (discr == 0)
		x0 = x1 = -0.5 * b / a;
	else
	{
		float q = (b > 0) ? -0.5 * (b + sqrt(discr)) : -0.5 * (b - sqrt(discr));
		x0 = q / a;
		x1 = c / q;
	}

	//设置x0 < x1
	if (x0 > x1)
		std::swap(x0, x1);

	return true;
}

//材质类型
enum MaterialType
{
	DIFFUSE_AND_GLOSSY,											//散射和光滑
	REFLECTION_AND_REFRACTION,									//反射和折射
	REFLECTION													//反射
};

//获得float类型随机数
inline float get_random_float()
{
	std::random_device dev;
	std::mt19937 rng(dev());
	std::uniform_real_distribution<float> dist(0.f, 1.f);		//随机数的取值范围[1, 6]

	return dist(rng);
}

//更新进度
inline void UpdateProgress(float progress)
{
	int barWidth = 70;

	std::cout << "[";
	int pos = barWidth * progress;
	for (int i = 0; i < barWidth; ++i)
	{
		if (i < pos)
			std::cout << "=";
		else if (i == pos)
			std::cout << ">";
		else
			std::cout << " ";
	}

	std::cout << "]" << int(progress * 100.0) << "%\r";
	std::cout.flush();
}
