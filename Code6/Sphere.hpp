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
        Vector3f center;                                        //球心
        float radius, radius2;                                  //半径和半径的平方
        Material* m;                                            //材质

    public:
        //构造函数
        Sphere(const Vector3f& c, const float& r) : center(c), radius(r), radius2(r * r), m(new Material()) {}

        //判断球体是否与射线相交
        bool intersect(const Ray& ray)
        {
            //anaytic solution(分析解决)
            Vector3f L = ray.origin - center;
            float a = dotProduct(ray.direction, ray.direction);
            float b = 2 * dotProduct(ray.direction, L);
            float c = dotProduct(L, L) - radius2;
            float t0, t1;
            if (!solveQuadratic(a, b, c, t0, t1)) return false;
            if (t0 < 0) t0 = t1;
            if (t0 < 0) return false;

            return true;
        }

        bool intersect(const Ray& ray, float& tnear, uint32_t& index) const
        {
            //analytic solution(分析解决)
            Vector3f L = ray.origin - center;
            float a = dotProduct(ray.direction, ray.direction);
            float b = 2 * dotProduct(ray.direction, L);
            float c = dotProduct(L, L) - radius2;
            float t0, t1;
            if (!solveQuadratic(a, b, c, t0, t1)) return false;
            if (t0 < 0) t0 = t1;
            if (t0 < 0) return false;
            tnear = t0;

            return true;
        }

        //获得射线与球体的相交点
        Intersection getIntersection(Ray ray)
        {
            Intersection result;
            result.happened = false;

            Vector3f L = ray.origin - center;
            float a = dotProduct(ray.direction, ray.direction);
            float b = 2 * dotProduct(ray.direction, L);
            float c = dotProduct(L, L) - radius2;
            float t0, t1;
            if (!solveQuadratic(a, b, c, t0, t1)) return result;
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

        //获得面的性质
        void getSurfaceProperties(const Vector3f& P, const Vector3f& I, const uint32_t& index, const Vector2f& uv, Vector3f& N, Vector2f& st) const
        {
            //相交点的法线
            N = normalize(P - center);
        }

        //重新计算散射颜色
        Vector3f evalDiffuseColor(const Vector2f& st) const
        {
            return m->getColor();
        }

        //获得包围盒
        Bounds3 getBounds()
        {
            return Bounds3(Vector3f(center.x - radius, center.y - radius, center.z - radius), 
                           Vector3f(center.x + radius, center.y + radius, center.z + radius));
        }
};

#endif //RAYTRACING_SPHERE_H
