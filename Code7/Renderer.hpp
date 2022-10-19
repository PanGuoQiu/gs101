#include "Scene.hpp"

#pragma once

//有效碰撞结构体
struct hit_payload
{
	float tNear;			//包围盒的近平面
	uint32_t index;			//索引
	Vector2f uv;			//纹理
	Object* hit_obj;		//碰撞的物体
};


//渲染器类
class Renderer
{
	public:
		void Render(const Scene& scene, int spp = 1);
};
