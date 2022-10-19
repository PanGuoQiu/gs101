#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// 程序里的主函数，我们创建场景(创建对象和光源)以及设置渲染选项 (图像宽度和高度，最大递归深度，视野，etc.)
// 然后，我们调用渲染函数
int main(int argc, char const *argv[])
{
    //场景
    Scene scene(1280, 960);

    //三角形网格
    MeshTriangle bunny("../models/bunny/bunny.obj");

    //增加场景物体对象和光源
    scene.Add(&bunny);                                              //增加一个兔子对象
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 1));   //一个光源
    scene.Add(std::make_unique<Light>(Vector3f(20, 70, 20), 1));    //一个光源
    scene.buildBVH();                                               //加速计算结构

    //实例化一个渲染器
    Renderer r;

    //渲染的时间
    auto start = std::chrono::system_clock::now();                  //系统当前时间：开始
    r.Render(scene);                                                //渲染场景
    auto stop = std::chrono::system_clock::now();                   //系统当前时间：结束

    //转换为显示时间
    std::cout << "完全渲染：\n";
    std::cout << "所需时间： " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count()   << "时\n";
    std::cout << "          " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << "分\n";
    std::cout << "          " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << "秒\n";

    return 0;
}