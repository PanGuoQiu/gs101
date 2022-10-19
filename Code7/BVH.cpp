#include <algorithm>
#include <cassert>
#include "BVH.hpp"

//构造函数
BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode, SplitMethod splitMethod)
                :  maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod), primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf("\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n", hrs, mins, secs);
}

//递归创建节点
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    //计算BVH节点中所有对象的包围盒
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());

    if (objects.size() == 1)
    {
        //创建BVH子节点
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2)
    {
        node->left = recursiveBuild(std::vector<Object *> {objects[0]});
        node->right = recursiveBuild(std::vector<Object *> {objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());

        //获得包围盒的最长轴
        int dim = centroidBounds.maxExtent();
        switch (dim)
        {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2){
                    return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;
                });
                break;

            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2){
                    return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;
                });
                break;

            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2){
                    return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;
                });
                break;
        }

        //左节点和右节点
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        //断言
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        //递归计算左节点和右节点的包围盒
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        //计算总包围盒和面积
        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }

    return node;
}

//计算射线与BVH的相交点
Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;

    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

//通过BVH节点和射线，获得相交点
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    //待办事项：通过BVH节点查找相交
    std::array<int, 3> dirisNeg{ray.direction.x < 0, ray.direction.y < 0, ray.direction.z < 0};

    if (node->bounds.IntersectP(ray, ray.direction_inv, dirisNeg))
    {
        if (node->left == nullptr && node->right == nullptr)
        {
            if (node->object)
                return node->object->getIntersection(ray);

            return Intersection();
        }

        //有相交点。判断是左边还是右边的节点
        Intersection left_inter, right_inter;
        left_inter = getIntersection(node->left, ray);
        right_inter = getIntersection(node->right, ray);
        if (!left_inter.happened)   return right_inter;
        if (!right_inter.happened)  return left_inter;
        if (left_inter.distance < right_inter.distance)
            return left_inter;
        else
            return right_inter;
    }

    return Intersection();
}

//获得采样点
void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection& pos, float& pdf)
{
    //结束条件
    if (node->left == nullptr || node->right == nullptr)
    {
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }

    //继续递归
    if (p < node->left->area)
        getSample(node->left, p, pos, pdf);
    else
        getSample(node->right, p - node->left->area, pos, pdf);
}

//计算采样点
void BVHAccel::Sample(Intersection& pos, float& pdf)
{
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}
