#pragma once

#include <vector>
#include "Vector.hpp"
#include "Object.hpp"
#include "Light.hpp"
#include "AreaLight.hpp"
#include "BVH.hpp"
#include "Ray.hpp"

//场景类
class Scene
{
    public:
        //setting up options(安装选项)
        int width = 1280;                                                   //宽度
        int height = 960;                                                   //高度
        double fov = 90;                                                    //垂直视角90度
        Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);   //背景颜色
        int maxDepth = 5;                                                   //最大深度

        //构造函数
        Scene(int w, int h) : width(w), height(h) {}

        //增加场景中的物体和光照
        void Add(Object* object) { objects.push_back(object); }
        void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

        //获得场景中的物体和光照
        const std::vector<Object*>& get_objects() const { return objects; }
        const std::vector<std::unique_ptr<Light> >& get_lights() const { return lights; }

        Intersection intersect(const Ray& ray) const;                       //射线与场景包围盒的相交点
        BVHAccel* bvh;                                                      //包围盒数据结构

        //创建包围盒数据结构
        void buildBVH();

        //路径追踪的实现
        Vector3f castRay(const Ray& ray, int depth) const;

        //追踪
        bool trace(const Ray& ray, const std::vector<Object*> &objects, float& tNear, uint32_t& index, Object** hitObject);

        //处理面积光
        std::tuple<Vector3f, Vector3f> HandleAreaLight(const AreaLight& light, const Vector3f& hitPoint, const Vector3f& N, 
                                                       const Vector3f& shadowPointOrig, 
                                                       const std::vector<Object *> &objects, uint32_t& index, 
                                                       const Vector3f& dir, float specularExponent);

        //creating the scene (adding objects and lights)创建场景(增加物体和光源)
        std::vector<Object *> objects;
        std::vector<std::unique_ptr<Light> > lights;

        //Compute reflection direction(计算反射方向)
        Vector3f reflect(const Vector3f& I, const Vector3f& N) const
        {
            return I - 2 * dotProduct(I, N) * N;
        }

        //Compute refraction direction using Snell's law            (使用Snell定律计算折射方向)
        //We need to handle with care the two possible situations:  我们需要小心处理这两种可能的情况:
        //  -When the ray is inside the object                      (当射线是在物体内部)
        //  -When the ray is outside                                (当射线在外面)
        //If the ray is outside, you need to make cosi positive cosi = -N · I                       (如果射线在外部，你需要使cosi为正，即cosi = -N·I)
        //If the ray is inside, you need to invert the refractive indices and negate the normal N   (如果射线是在物体内，你需要反转折射率和否定法线N)
        Vector3f refract(const Vector3f& I, const Vector3f& N, const float& ior) const
        {
            float cosi = clamp(-1, 1, dotProduct(I, N));
            float etai = 1, etat = ior;
            Vector3f n = N;
            if (cosi < 0)
            {
                cosi = -cosi;
            }
            else
            {
                std::swap(etai, etat);
                n = -N;
            }

            float eta = etai / etat;
            float k = 1 - eta * eta * (1 - cosi * cosi);

            return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
        }

        //Conpute Fresnel equation                          (计算菲涅尔方程)
        //\param I is the incident view direction           (参数I 是入射视图方向)
        //\param N is the normal at the intersection point  (参数N 是相交点的法向量)
        //\param ior is the material refractive index       (参数ior 是材质的折射率)
        //\param [out] kr is the amount of light reflected  (返回参数kr 是反射光的量)
        void fresnel(const Vector3f& I, const Vector3f& N, const float& ior, float& kr) const
        {
            float cosi = clamp(-1, 1, dotProduct(I, N));
            float etai = 1, etat = ior;
            if (cosi > 0) 
            {
                std::swap(etai, etat);
            }

            //Compute sini using Snell's law (使用Snell定律计算sini)
            float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
            //total internal reflection (全内反射)
            if (sint >= 1)
            {
                kr = 1;
            }
            else
            {
                float cost = sqrtf(std::max(0.f, 1 - sint * sint));
                cosi = fabsf(cosi);
                float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
                float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
                kr = (Rs * Rs + Rp * Rp) / 2;
            }

            //As a consequence of the conservation of energy, transmittance is given by: 
            //由于能量守恒，透光率为:
            //kt = 1 - kr;
        }
};
