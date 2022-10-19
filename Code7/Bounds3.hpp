#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H

#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

//3维包围盒类
class Bounds3
{
    public:
        //各种构造函数
        Bounds3()
        {
            double minNum = std::numeric_limits<double>::lowest();  //取类型的最小值
            double maxNum = std::numeric_limits<double>::max();     //取类型的最大值

            //最大值设为最小值和最小值设为最大
            pMax = Vector3f(minNum, minNum, minNum);
            pMin = Vector3f(maxNum, maxNum, maxNum);
        }

        //重载构造函数
        Bounds3(const Vector3f p) : pMin(p), pMax(p) {}

        //重载构造函数，自动计算最小值和最大值
        Bounds3(const Vector3f p1, const Vector3f p2)
        {
            pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
            pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
        }

        //计算包围盒对角线的向量
        Vector3f Diagonal() const
        {
            return pMax - pMin;
        }

        //判断对角线各轴的最长轴的索引
        int maxExtent() const
        {
            Vector3f d = Diagonal();                            //计算包围盒的对角线
            if (d.x > d.y && d.x > d.z)                         //x轴最长
                return 0;
            else if (d.y > d.z)                                 //y轴最长
                return 1;
            else                                                //z轴最长
                return 2;
        }

        //计算包围盒的表面积
        double SurfaceArea() const
        {
            Vector3f d = Diagonal();
            return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
        }

        //包围盒的中心点
        Vector3f Centroid()
        {
            return 0.5 * pMin + 0.5 * pMax;
        }

        //包围盒相交
        Bounds3 Intersect(const Bounds3& b)
        {
            return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y), fmax(pMin.z, b.pMin.z)), 
                           Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y), fmin(pMax.z, b.pMax.z)));
        }

        //包围盒的抵消
        Vector3f Offset(const Vector3f& p) const
        {
            Vector3f o = p - pMin;
            if (pMax.x > pMin.x)
                o.x /= pMax.x - pMin.x;
            if (pMax.y > pMin.y)
                o.x /= pMax.y - pMin.y;
            if (pMax.z > pMin.z)
                o.z /= pMax.z - pMin.z;

            return o;
        }

        //包围盒是否的重叠
        bool Overlaps(const Bounds3& b1, const Bounds3& b2)
        {
            bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
            bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
            bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);

            return (x && y && z);
        }

        //判断一个点是否在包围盒内
        bool Inside(const Vector3f& p, const Bounds3& b)
        {
            return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y && p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
        }

        //重载[]运算符的索引，0返回最小值、其他返回最大值
        inline const Vector3f& operator[](int i) const
        {
            return (i == 0) ? pMin : pMax;
        }

        //射线和包围盒的相交点
        inline bool IntersectP(const Ray& ray, const Vector3f& invDir, const std::array<int, 3>& dirIsNeg) const;

    public:
        Vector3f pMin, pMax;                                    //指定包围盒的两个顶点：最小值和最大值
};

//射线和包围盒的相交点(射线与平面相交)
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir, const std::array<int, 3>& dirIsNeg) const
{
    //invDir: 射线方向(x, y, z)，invDir = (1.0/x, 1.0/y, 1.0/z)，因为使用乘法比除法更快
    //dirIsNeg: 射线方向(x, y, z)，dirIsNeg = [int(x>0), int(y>0), int(z>0)]，用这个来简化你的逻辑
    //待办事项：测试射线边界是否相交
    float t_min_x = (pMin.x - ray.origin.x) * invDir.x;
    float t_min_y = (pMin.y - ray.origin.y) * invDir.y;
    float t_min_z = (pMin.z - ray.origin.z) * invDir.z;

    float t_max_x = (pMax.x - ray.origin.x) * invDir.x;
    float t_max_y = (pMax.y - ray.origin.y) * invDir.y;
    float t_max_z = (pMax.z - ray.origin.z) * invDir.z;

    if (dirIsNeg[0]) std::swap(t_min_x, t_max_x);
    if (dirIsNeg[1]) std::swap(t_min_y, t_max_y);
    if (dirIsNeg[2]) std::swap(t_min_z, t_max_z);

    //进入时：最小值取"最大时间值"；出去时：最大值取"最小时间值"
    float t_min = std::max(t_min_x, std::max(t_min_y, t_min_z));
    float t_max = std::min(t_max_x, std::min(t_max_y, t_max_z));

    if (t_max >= 0 && t_min <= t_max)
        return true;

    return false;
}

//并集 (包围盒 并 包围盒)
inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;

    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);

    return ret;
}

//并集 (包围盒 并 点)
inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;

    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);

    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
