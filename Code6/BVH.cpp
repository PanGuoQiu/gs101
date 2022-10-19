#include <algorithm>
#include <cassert>
#include "BVH.hpp"

//构造函数
BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode, SplitMethod splitMethod) 
                    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod), primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);                          //设置root为递归创建物体的第一个对象，并计算这段时间

    time(&stop);
    double diff = difftime(stop, start);                        //开始到结束的时间段
    int hrs = (int)diff / 3600;                                 //转换为多少小时
    int mins = ((int)diff / 60) - (hrs * 60);                   //转换为多少分钟
    int secs = (int)diff - (hrs * 3600) - (mins * 60);          //转换为多少秒

    //生成完整的BVH： 所需时间：
    printf("\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n", hrs, mins, secs);
}

//递归创建BVH，获得BVH节点指针
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();                    //创建BVH节点指针

    //Compute bounds of all primitives in BVH node(计算BVH节点中所有对象的包围盒)
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds =Union(bounds, objects[i]->getBounds());
    
    if (objects.size() == 1)
    {
        //Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)
    {
        node->left = recursiveBuild(std::vector<Object *>{objects[0]});
        node->right = recursiveBuild(std::vector<Object *>{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 centriodBounds;
        for (int i = 0; i < objects.size(); ++i)
            centriodBounds = Union(centriodBounds, objects[i]->getBounds().Centroid());
        
        //寻找最长轴进行细分
        int dim = centriodBounds.maxExtent();
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

        //每次递归，场景中的叶节点包围盒细分为两部分
        auto beginning = objects.begin();                               //第一个对象
        auto midding = objects.begin() + (objects.size() / 2);          //中间对象
        auto ending = objects.end();                                    //最后一个对象

        auto leftshapes = std::vector<Object*>(beginning, midding);     //左形状
        auto rightshapes = std::vector<Object*>(midding, ending);       //右形状

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

//获得BVH节点与射线的相交的点
Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    
    isect = BVHAccel::getIntersection(root, ray);

    return isect;
}

//获得射线与BVH节点的相交点
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    //待办事项：通过BVH节点查找相交
    std::array<int , 3> dirisNeg{ray.direction.x < 0, ray.direction.y < 0, ray.direction.z < 0};

    //判断射线是否与包围盒相交
    if (node->bounds.IntersectP(ray, ray.direction_inv, dirisNeg))
    {
        //判断左节点和右节点是否同时为空，如果是，则返回射线与世界包围盒的相交点
        if (node->left == nullptr && node->right == nullptr)
        {
            if (node->object)
                return node->object->getIntersection(ray);

            return Intersection();
        }

        //如果不是，则分别计算左节点或右节点的相交点
        Intersection left_inter, right_inter;
        left_inter = getIntersection(node->left, ray);
        right_inter = getIntersection(node->right, ray);

        if (!left_inter.happened) return right_inter;
        if (!right_inter.happened) return left_inter;
        
        if (left_inter.distance < right_inter.distance)
            return left_inter;
        else
            return right_inter;
    }

    return Intersection();
}
