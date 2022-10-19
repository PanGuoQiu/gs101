#include "Scene.hpp"

//创建BVH数据结构
void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

//计算射线与场景的相交点
Intersection Scene::intersect(const Ray& ray) const
{
    return this->bvh->Intersect(ray);
}

//光线采样
void Scene::sampleLight(Intersection& pos, float& pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }

    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

//判断追踪的光线是否与场景中的对象相交
bool Scene::trace(const Ray& ray, const std::vector<Object*>& objects, float& tNear, uint32_t& index, Object** hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvk;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

//阴影
Vector3f Scene::shade(const Ray& ray) const
{
    //返回Vector3f(1.0f, 1.0f, 1.0f);
    Vector3f direct_energy(0.f, 0.f, 0.f);
    Vector3f bounced_energy(0.f, 0.f, 0.f);
    Intersection pos_on_obj = intersect(ray);
    if (!pos_on_obj.happened)
        return direct_energy;

    Intersection pos_on_ls;
    float pdf_ls;
    sampleLight(pos_on_ls, pdf_ls);
    //这里我们需要使用hasEmit函数
    if (pos_on_obj.obj->hasEmit())
        return pos_on_ls.emit;

    //曝光
    Ray direct_ray = Ray(pos_on_obj.coords, normalize(pos_on_ls.coords - pos_on_obj.coords));

    //检查光线是否被其他物体遮挡
    Intersection blocked = intersect(direct_ray);

    if (blocked.happened && blocked.obj->hasEmit())
    {
        //光源的双向反射分布函数
        //这里cos_theta为0
        Vector3f f_r_ls = pos_on_obj.m->eval(-ray.direction, direct_ray.direction, pos_on_obj.normal);
        float cos_theta = std::max(0.0f, dotProduct(direct_ray.direction, pos_on_obj.normal));
        float cos_theta_ = std::max(0.0f, dotProduct(-direct_ray.direction, pos_on_ls.normal));
        direct_energy = pos_on_ls.emit * f_r_ls * cos_theta * cos_theta_ / squaredNorm(pos_on_ls.coords - pos_on_obj.coords) / pdf_ls;
    }

    // Use russian Roulette to determine whether we should terminate the process
    // 使用俄罗斯轮盘来决定我们是否应该终止这个过程
    // float p = get_random_float();
    // if(p < RussianRoulette && !pos_on_obj.obj->hasEmit() )
    // {
    //     Vector3f w_o = pos_on_obj.m->sample(- ray.direction, pos_on_obj.normal).normalized();
    //     Vector3f f_r_obj = pos_on_obj.m->eval(- ray.direction, w_o, pos_on_obj.normal);
    //     float pdf_obj = pos_on_obj.m->pdf(- ray.direction, w_o, pos_on_obj.normal);
    //     Ray next_ray(pos_on_obj.coords, w_o);
    //     Vector3f radiance = shade(next_ray);
    //     float cos_theta = std::max(0.0f, dotProduct(next_ray.direction, pos_on_obj.normal));
    //     bounced_energy +=  radiance * f_r_obj * cos_theta / (pdf_obj + EPSILON) / RussianRoulette;
    // }
    // 使用俄罗斯轮盘来决定我们是否应该终止这个过程
     float p = get_random_float();
     if(p < RussianRoulette && !pos_on_obj.obj->hasEmit() )
     {
         Vector3f w_o = pos_on_obj.m->sample(- ray.direction, pos_on_obj.normal).normalized();
         Vector3f f_r_obj = pos_on_obj.m->eval(- ray.direction, w_o, pos_on_obj.normal);
         float pdf_obj = pos_on_obj.m->pdf(- ray.direction, w_o, pos_on_obj.normal);
         Ray next_ray(pos_on_obj.coords, w_o);
         Vector3f radiance = shade(next_ray);
         float cos_theta = std::max(0.0f, dotProduct(next_ray.direction, pos_on_obj.normal));
         bounced_energy +=  radiance * f_r_obj * cos_theta / (pdf_obj + EPSILON) / RussianRoulette;
    }
    
    return direct_energy + bounced_energy;
}

//路径追踪的实现
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    //代办事项：在这里实现路径追踪算法
    //第一：我们计算相交点，如果光线没有击中任何物体，返回0
    //第二：我们对光线进行采样并计算pdf
    return shade(ray);
}
