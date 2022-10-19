#ifndef RAYTRACING_SPHERE_H
#define RAYTRACING_SPHERE_H

#include "Object.hpp"
#include "Vector.hpp"
#include "Bounds3.hpp"
#include "Material.hpp"

//球体类
class Sphere : public Object
{
    public:
        //构造函数
        Sphere(const Vector3f& c, const float& r, Material* mt = new Material()) : center(c), radius(r), radius2(r * r), m(mt), area(4 * M_PI * r * r) {}

        //射线是否与球体相交
        bool intersect(const Ray& ray)
        {
            //分析解决
            Vector3f L = ray.origin - center;                   //射线原点到球心的距离
            float a = dotProduct(ray.direction, ray.direction);
            float b = 2 * dotProduct(ray.direction, L);
            float c = dotProduct(L, L) - radius2;
            float t0, t1;
            float area = 4 * M_PI * radius2;
            if (!solveQuadratic(a, b, c, t0, t1))
                return false;

            if (t0 < 0) t0 = t1;
            if (t0 < 0) return false;

            return true;
        }

        //重载射线是否与球体相交
        bool intersect(const Ray& ray, float& tnear, uint32_t& index) const
        {
            //分析解决
            Vector3f L = ray.origin - center;
            float a = dotProduct(ray.direction, ray.direction);
            float b = 2 * dotProduct(ray.direction, L);
            float c = dotProduct(L, L) - radius2;
            float t0, t1;
            if (!solveQuadratic(a, b, c, t0, t1))
                return false;

            if (t0 < 0) t0 = t1;
            if (t0 < 0) return false;
            tnear = t0;

            return true;
        }

        //获得相交点
        Intersection getIntersection(Ray ray)
        {
            Intersection result;
            result.happened = false;
            Vector3f L = ray.origin - center;
            float a = dotProduct(ray.direction, ray.direction);
            float b = 2 * dotProduct(ray.direction, L);
            float c = dotProduct(L, L) - radius2;
            float t0, t1;
            if (!solveQuadratic(a, b, c, t0, t1))
                return result;

            if (t0 < 0) t0 = t1;
            if (t0 < 0) return result;
            result.happened = true;

            result.coords = Vector3f(ray.origin + ray.direction * t0);
            result.normal = normalize(Vector3f(result.coords - center));
            result.m = this->m;
            result.obj = this;
            result.distance = t0;

            return result;
        }

        //获得表面的法线
        void getSurfaceProperties(const Vector3f& P, const Vector3f& I, const uint32_t& index, const Vector2f& uv, Vector3f& N, Vector2f& st) const
        {
            N = normalize(P - center);
        }

        //重新计算散射颜色
        Vector3f evalDiffuseColor(const Vector2f& st) const
        {
            //return m->getColor();
        }

        //获得包围盒
        Bounds3 getBounds()
        {
            return Bounds3(Vector3f(center.x - radius, center.y - radius, center.z - radius), 
                           Vector3f(center.x + radius, center.y + radius, center.z + radius));
        }

        //计算采样点
        void Sample(Intersection& pos, float& pdf)
        {
            float theta = 2.0 * M_PI * get_random_float(), phi = M_PI * get_random_float();
            Vector3f dir(std::cos(phi), std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta));
            pos.coords = center + radius * dir;
            pos.normal = dir;
            pos.emit = m->getEmission();
            pdf = 1.0f / area;
        }

        //获得球体面积
        float getArea()
        {
            return area;
        }

        //判断物体是否有自发光
        bool hasEmit()
        {
            return m->hasEmission();
        }

    public:
        Vector3f center;                                        //球心
        float radius, radius2;                                  //半径、半径的平方
        Material* m;                                            //球体的材质
        float area;                                             //球体的表面积
};

#endif //RAYTRACING_SPHERE_H
