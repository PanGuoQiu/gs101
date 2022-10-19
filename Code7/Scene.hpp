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
        //安装选项
        int width = 1280;
        int height = 960;
        double fov = 40;
        Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
        int maxDepth = 1;
        float RussianRoulette = 0.8;

        //构造函数
        Scene(int w, int h) : width(w), height(h) {}

        //增加场景中的对象和光源
        void Add(Object* object)
        {
            objects.push_back(object);
        }

        void Add(std::unique_ptr<Light> light)
        {
            lights.push_back(std::move(light));
        }

        //获得对象数组指针
        const std::vector<Object*>& get_objects() const
        {
            return objects;
        }

        //获得光源数组指针
        const std::vector<std::unique_ptr<Light> >& get_lights() const
        {
            return lights;
        }

        Intersection intersect(const Ray& ray) const;               //计算射线与场景的相交点
        BVHAccel* bvh;                                              //BVH结构体
        void buildBVH();                                            //创建BVH数据结构
        Vector3f castRay(const Ray& ray, int depth) const;          //路径光线的实现
        void sampleLight(Intersection& pos, float& pdf) const;      //光线的采样点

        //追踪光线是否与场景中的对象相交
        bool trace(const Ray& ray, const std::vector<Object*> &objects, float& tNear, uint32_t& index, Object** hitObject);

        //处理面积光
        std::tuple<Vector3f, Vector3f> HandleAreaLight(const AreaLight& light, const Vector3f& hitPoint, const Vector3f& N, 
                                                       const Vector3f& shadowPointOrig, const std::vector<Object*>& objects, uint32_t& index,
                                                       const Vector3f& dir, float specularExponent);

        //阴影
        Vector3f shade(const Ray& ray) const;

        //创建场景(增加对象和光源)
        std::vector<Object*> objects;
        std::vector<std::unique_ptr<Light> > lights;

        //计算反射方向
        Vector3f reflect(const Vector3f& I, const Vector3f& N) const
        {
            return I - 2 * dotProduct(I, N) * N;
        }

        //使用斯涅尔定律计算折射方向
        //我们需要小心处理这两种可能的情况：
        //  当射线是在对象内部
        //  当射线是在对象外部
        //如果射线在外部，你需要使cosi为正，即cosi = -N·I
        //如果射线是在物体内，你需要反转折射率和否定法线N
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

        //计算菲涅尔方程
        //参数I 是入射视图方向
        //参数N 是相交点的法向量
        //参数ior 是材质的折射率
        //返回参数kr 是反射光的量
        void fresnel(const Vector3f& I, const Vector3f& N, const float& ior, float& kr) const
        {
            float cosi = clamp(-1, 1, dotProduct(I, N));
            float etai = 1, etat = ior;
            if (cosi > 0)
            {
                std::swap(etai, etat);
            }

            //使用斯涅尔定律计算sini
            float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
            //全内反射
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

            //由于能量守恒，透光率为：
            //kt = 1 - kr;
        }
};
