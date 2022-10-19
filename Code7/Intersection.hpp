#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H

#include "Vector.hpp"
#include "Material.hpp"

//类的前置声明
class Object;
class Sphere;

//相交点结构体
struct Intersection
{
    //构造函数
    Intersection()
    {
        happened = false;
        coords = Vector3f();
        normal = Vector3f();
        distance = std::numeric_limits<double>::max();
        obj = nullptr;
        m = nullptr;
    }

    //相交点的属性变量
    bool happened;              //是否发生碰撞
    Vector3f coords;            //碰撞点坐标
    Vector3f tcoords;           //碰撞点对应的纹理坐标
    Vector3f normal;            //碰撞点的法线
    Vector3f emit;              //碰撞点的对象自发光的位置
    double distance;            //射线原点与碰撞点的距离
    Object* obj;                //射线与对象碰撞的对象
    Material* m;                //对象的材质
};

#endif //RAYTRACING_INTERSECTION_H

