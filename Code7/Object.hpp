#pragma once
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

        //判断射线与对象是否相交
        virtual bool intersect(const Ray& ray) = 0;
        virtual bool intersect(const Ray& ray, float &, uint32_t &) const = 0;

        //获得相交点
        virtual Intersection getIntersection(Ray _ray) = 0;
        //获得表面属性
        virtual void getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &, const Vector2f &, Vector3f &, Vector2f &) const = 0;
        //重新计算散射颜色
        virtual Vector3f evalDiffuseColor(const Vector2f &) const = 0;
        //获得对象包围盒
        virtual Bounds3 getBounds() = 0;
        //获得面积
        virtual float getArea() = 0;
        //计算采样点
        virtual void Sample(Intersection& pos, float& pdf) = 0;
        //对象是否有自发光
        virtual bool hasEmit() = 0;
};


#endif //RAYTRACING_OBJECT_H
