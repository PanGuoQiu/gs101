#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct BVHBuildNode;                    //BVH节点
//BVHAccel类的前置声明
struct BVHPrimitiveInfo;                //BVH的原始信息

//BVHAccel类声明
//inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel
{
    public:
        //BVHAccel公共类型：细分
        enum class SplitMethod { NAIVE, SAH };

        //BVHAccel公共方法：
        //构造函数和析构函数：物体对象、最大节点数、细分方法
        BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
        ~BVHAccel();

        //获得世界包围盒
        Bounds3 WorldBound() const;

        //计算相交点
        Intersection Intersect(const Ray& ray) const;
        //获得相交点
        Intersection getIntersection(BVHBuildNode* node, const Ray& ray) const;
        //判断射线是否与BVH相交
        bool IntersectP(const Ray& ray) const;

        BVHBuildNode* root;

        //BVHAccel类私有方法：
        BVHBuildNode* recursiveBuild(std::vector<Object*> objects);                 //递归创建包含物体的节点

        //BVHAccel私有数据
        const int maxPrimsInNode;                               //最大节点数
        const SplitMethod splitMethod;                          //细分方法
        std::vector<Object*> primitives;                        //原始对象指针数组

        //获得采样点
        void getSample(BVHBuildNode* node, float p, Intersection& pos, float& pdf);
        void Sample(Intersection& pos, float& pdf);
};

//BVH节点结构体
struct BVHBuildNode
{
    Bounds3 bounds;                                                 //包围盒
    BVHBuildNode* left;                                             //左边节点
    BVHBuildNode* right;                                            //右边节点
    Object* object;                                                 //节点存储的物体对象
    float area;                                                     //面积

    public:
        int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;    //轴细分、第一个节点数、对象数

        //构造函数
        BVHBuildNode()
        {
            bounds = Bounds3();
            left = nullptr;
            right = nullptr;
            object = nullptr;
        }
};

#endif //RAYTRACING_BVH_H
