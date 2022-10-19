#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Light.hpp"
#include "Renderer.hpp"

//程序的主函数，我们创建一个场景(创建对象和光源)
//以及为渲染设置选项(图像宽度和高度，最大递归深度、视野、etc)，我们再调用渲染函数
int main()
{
	Scene scene(1280, 960);

	auto sph1 = std::make_unique<Sphere>(Vector3f(-1, 0, -12), 2);			//第一个球体对象
	sph1->materialType = DIFFUSE_AND_GLOSSY;								//材质
	sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8);							//散射颜色
	
	auto sph2 = std::make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5);		//第二个球体对象
	sph2->ior = 1.5;														//折射率
	sph2->materialType = REFLECTION_AND_REFRACTION;							//材质

	scene.Add(std::move(sph1));												//场景增加第一个球体对象
	scene.Add(std::move(sph2));												//场景增加第二个球体对象

	//底面
	Vector3f verts[4] = { {-5, -3, -6}, {5, -3, -6}, {5, -3, -16}, {-5, -3, -16} };
	uint32_t vertIndex[6] = { 0, 1, 3, 1, 2, 3 };
	Vector2f st[4] = { {0, 0}, {1, 0}, {1, 1}, {0, 1} };
	auto mesh = std::make_unique<MeshTriangle>(verts, vertIndex, 2, st);
	mesh->materialType = DIFFUSE_AND_GLOSSY;

	scene.Add(std::move(mesh));												//场景增加底面网格
	scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 0.5));			//场景增加第一个光源
	scene.Add(std::make_unique<Light>(Vector3f(30, 50, -12), 0.5));			//场景增加第二个光源

	//声明一个渲染器，并渲染场景
	Renderer r;
	r.Render(scene);

	return 0;
}
