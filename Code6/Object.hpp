#ifndef RAYTRACING_OBJECT_H
#define RAYTRACING_OBJECT_H

#include "Vector.hpp"
#include "global.hpp"
#include "Bounds3.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"

//对象类
class Object
{
    public:
        //构造函数和虚析构函数
        Object() {}
        virtual ~Object() {}

        //射线是否与物体相交点
        virtual bool intersect(const Ray& ray) = 0;
        virtual bool intersect(const Ray& ray, float&, uint32_t&) const = 0;

        //获得相交点
        virtual Intersection getIntersection(Ray _ray) = 0;
        //获得面的性质
        virtual void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t&, const Vector2f&, Vector3f&, Vector2f&) const = 0;
        //重新计算散射颜色
        virtual Vector3f evalDiffuseColor(const Vector2f&) const = 0;
        //获得对象的包围盒
        virtual Bounds3 getBounds() = 0;
};

#endif //RAYTRACING_OBJECT_H
