#pragma once 

#include "Scene.hpp"

//有效碰撞结构体
struct hit_payload
{
	float tNear;							//近面
	uint32_t index;							//索引
	Vector2f uv;							//纹理坐标
	Object* hit_obj;						//碰撞的物体
};

//渲染器类
class Renderer
{
	public:
		//渲染
		void Render(const Scene& scene);	//参数：scene场景
};
