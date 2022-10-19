#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"
#include "Bounds3.hpp"
#include "Vector.hpp"

struct BVHBuildNode;                                                            //BVH节点
//BVHAccel类的前置声明
struct BVHPrimitiveInfo;                                                        //BVH的原始信息

//BVHAccel类声明 (加速 处理 BVH数据结构)
//static int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel
{
    public:
        //BVHAccel 公共类型
        enum class SplitMethod { NAIVE, SAH };

        //BVHAccel 公共方法   物体对象、最大节点数、细分方法
        BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE); //构造函数
        Bounds3 WorldBound() const;                                             //世界包围盒
        ~BVHAccel();                                                            //析构函数

        Intersection Intersect(const Ray& ray) const;
        Intersection getIntersection(BVHBuildNode* node, const Ray& ray) const; //获得相交点
        bool IntersectP(const Ray& ray) const;                                  //是否相交
        BVHBuildNode* root;                                                     //根节点

        //BVHAccel 私有方法
        BVHBuildNode* recursiveBuild(std::vector<Object*> objects);             //递归创建

        //BVHAccel 私有数据
        const int maxPrimsInNode;                                               //最大节点
        const SplitMethod splitMethod;                                          //细分方法
        std::vector<Object*> primitives;                                        //原始对象数据指针
};

//创建BVH节点结构体
struct BVHBuildNode
{
    Bounds3 bounds;                                                             //包围盒
    BVHBuildNode* left;                                                         //左边节点
    BVHBuildNode* right;                                                        //右边节点
    Object* object;                                                             //物体对象指针

    public:
        int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;
        //BVHBuildNode 公共方法
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
