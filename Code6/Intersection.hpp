#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H

#include "Vector.hpp"
#include "Material.hpp"

//前置类声明：对象类和球体类
class Object;
class Sphere;

//相交点类
struct Intersection
{
    Intersection()
    {
        happened = false;
        coords = Vector3f();
        normal = Vector3f();
        distance = std::numeric_limits<double>::max();
        obj = nullptr;
        m = nullptr;
    }

    bool happened;                                              //是否发生碰撞
    Vector3f coords;                                            //碰撞点坐标
    Vector3f normal;                                            //碰撞点法线
    double distance;                                            //射线原点与碰撞点的距离
    Object* obj;                                                //射线与对象碰撞的对象
    Material* m;                                                //材质
};

#endif //RAYTRACING_INTERSECTION_H
