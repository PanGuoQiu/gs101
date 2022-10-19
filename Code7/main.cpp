#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

//程序里的主函数，我们创建场景(创建对象和光源)以及设置渲染选项(图像宽度和高度，最大递归深度，视野，等等.)
//然后，我们调研渲染函数
int main(int argc, char** argv)
{
    //在这里更改定义及分辨率
    Scene scene(784, 784);
    int spp = 1;
    if (argc > 1) 
        spp = atoi(argv[1]);

    //红色的材质
    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    //绿色材质
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    //白色材质
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    //光源材质属性：散射、自发光
    Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) + 
                                             15.6f * Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) + 
                                             18.4f * Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));
    light->Kd = Vector3f(0.65f);

    //载入不同的三角形网格
    MeshTriangle floor("../models/cornellbox/floor.obj", white);            //白色的地面
    MeshTriangle shortbox("../models/cornellbox/shortbox.obj", white);      //白色的矮盒子
    MeshTriangle tallbox("../models/cornellbox/tallbox.obj", white);        //白色的高盒子
    MeshTriangle left("../models/cornellbox/left.obj", red);                //红色的左面墙
    MeshTriangle right("../models/cornellbox/right.obj", green);            //绿色的右面墙
    MeshTriangle light_("../models/cornellbox/light.obj", light);           //自发光光源

    //把对象增加到场景中
    scene.Add(&floor);
    
    scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);

    //根据场景创建BVH数据结构
    scene.buildBVH();

    //声明渲染器
    Renderer r;

    //计算渲染所需要的时间，并显示出来
    auto start = std::chrono::system_clock::now();
    r.Render(scene, spp);
    auto stop = std::chrono::system_clock::now();

    std::cout<<"渲染完成：\n";
    std::cout<<"实际时间："<<std::chrono::duration_cast<std::chrono::hours>(stop - start).count()<<"hours\n";
    std::cout<<"         "<<std::chrono::duration_cast<std::chrono::minutes>(stop - start).count()<<"minutes\n";
    std::cout<<"         "<<std::chrono::duration_cast<std::chrono::seconds>(stop - start).count()<<"seconds\n";

    return 0;
}
