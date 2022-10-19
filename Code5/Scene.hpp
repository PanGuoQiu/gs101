#pragma once

#include <memory>
#include "Vector.hpp"
#include "Object.hpp"
#include "Light.hpp"

//场景类
class Scene
{
	public:
		//设置属性
		int width = 1280;													//屏幕宽度
		int height = 960;													//屏幕高低
		double fov = 90;													//垂直视角大小
		Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);	//背景颜色
		int maxDepth = 5;													//最大深度
		float epsilon = 0.00001;											//浮点型比较系数(误差)

		//构造函数
		Scene(int w, int h) : width(w), height(h) {}

		//场景中增加对象
		void Add(std::unique_ptr<Object> object)
		{
			objects.push_back(std::move(object));
		}

		//场景中增加光源
		void Add(std::unique_ptr<Light> light)
		{
			lights.push_back(std::move(light));
		}

		//获得对象和光源
		[[nodiscard]] const std::vector<std::unique_ptr<Object>>& get_objects() const { return objects; }
		[[nodiscard]] const std::vector<std::unique_ptr<Light>>& get_lights() const { return lights; }

	private:
		//场景中对象和光源
		std::vector<std::unique_ptr<Object>> objects;						//对象数组指针
		std::vector<std::unique_ptr<Light>> lights;							//光源数组指针
};
