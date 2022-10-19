#include "Scene.hpp"

//创建BVH数据结构
void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

//获得相交点
Intersection Scene::intersect(const Ray& ray) const
{
    return this->bvh->Intersect(ray);
}

//光线追踪 /射线、对象指针、近点、索引、对象指针相交点指针
bool Scene::trace(const Ray& ray, const std::vector<Object*> &objects, float& tNear, uint32_t& index, Object** hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Whitted-style光线传输算法的实现：(E [S*] (D|G) L)
// 这个函数是由位置和方向定义的光线来计算相交点的颜色的函数。 注意，因此函数是递归的(它调用它自己)
// 如果相交物体的材质是反射或反射和折射的。然后我们计算反射/折射方向，并通过递归调用castRay()函数向场景投射两道新光线。
// 当表面是透明的，我们使用菲涅尔方程的结果混合反射和折射的颜色(它根据表面法线计算反射和折射的量，入射的视图方向和表面折射率)
//
// 如果表面是弥漫的或光滑的，我们使用Phong光照模型来计算相交点的颜色
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    if (depth > this->maxDepth)
    {
        return Vector3f(0.0, 0.0, 0.0);
    }

    Intersection intersection = Scene::intersect(ray);
    Material* m = intersection.m;
    Object* hitObject = intersection.obj;
    Vector3f hitColor = this->backgroundColor;
    //float tnear = kInfinity;
    
    Vector2f uv;
    uint32_t index = 0;
    if (intersection.happened)
    {
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal;           //normal
        Vector2f st;                                //st coordinates
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);
        //Vector3f tmp = hitPoint;
        switch (m->getType())
        {
            case REFLECTION_AND_REFRACTION:
            {
                Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
                Vector3f refractionDirection = normalize(refract(ray.direction, N, m->ior));
                Vector3f reflectionRayOrig = (dotProduct(refractionDirection, N) < 0) ? hitPoint - N * EPSILON : hitPoint + N * EPSILON;
                Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ? hitPoint - N * EPSILON : hitPoint + N * EPSILON;
                Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
                Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);
                float kr;
                fresnel(ray.direction, N, m->ior, kr);
                hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                break;
            }
            case REFLECTION:
            {
                float kr;
                fresnel(ray.direction, N, m->ior, kr);
                Vector3f reflectionDirection = reflect(ray.direction, N);
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? hitPoint + N * EPSILON : hitPoint - N * EPSILON;

                hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1) * kr;
                break;
            }
            default:
            {
                //[comment]
                //我们在默认情况下使用Phong光照模型。Phong模型是由漫反射和镜面反射组成
                //[/comment]
                Vector3f lightAmt = 0, specularColor = 0;
                Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ? hitPoint + N * EPSILON : hitPoint - N * EPSILON;

                //[comment]
                //循环遍历场景中的所有灯光，并总结它们的贡献
                //我们并且应用朗伯余弦定律
                //[/comment]
                for (uint32_t i = 0; i < get_lights().size(); ++i)
                {
                    auto area_ptr = dynamic_cast<AreaLight *>(this->get_lights()[i].get());
                    if (area_ptr)
                    {
                        // Do nothing for this assignment(什么都不做)
                    }
                    else
                    {
                        Vector3f lightDir = get_lights()[i]->position - hitPoint;
                        //光源和相交点之间距离的平方
                        float lightDistance2 = dotProduct(lightDir, lightDir);
                        lightDir = normalize(lightDir);
                        float LdotN = std::max(0.f, dotProduct(lightDir, N));
                        Object* shadowHitObject = nullptr;
                        float tNearShadow = kInfinity;
                        //is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                        //
                        bool inShadow = bvh->Intersect(Ray(shadowPointOrig, lightDir)).happened;
                        lightAmt += (1 - inShadow) * get_lights()[i]->intensity * LdotN;
                        Vector3f reflectionDirection = reflect(-lightDir, N);
                        specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)), m->specularExponent) * get_lights()[i]->intensity;
                    }
                }

                hitColor = lightAmt * (hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks);
                break;
            }
        }
    }

    return hitColor;
}
