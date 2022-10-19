#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H

#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

//3维空间中：包围盒类
class Bounds3
{
    public:
        Vector3f pMin, pMax;                                        //包围盒指定的两个点：最小值和最大值

        //构造函数
        Bounds3()
        {
            double minNum = std::numeric_limits<double>::lowest();	//获得double类型的最小值
            double maxNum = std::numeric_limits<double>::max();	    //获得double类型的最大值
            pMax = Vector3f(minNum, minNum, minNum);		        //最大变量设置为最小值
            pMin = Vector3f(maxNum, maxNum, maxNum);		        //最小变量设置为最大值
        }

        //重载构造函数
        Bounds3(const Vector3f p) : pMin(p), pMax(p) {}

        //重载构造函数
        Bounds3(const Vector3f p1, const Vector3f p2)
        {
            pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
            pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
        }

        //对角线向量
        Vector3f Diagonal() const
        {
            return pMax - pMin;
        }

        //最大范围
        int maxExtent() const
        {
            Vector3f d = Diagonal();	                        //对角线向量
	        //判断对角线向量分量的最大值
            if (d.x > d.y && d.x > d.z)
                return 0;
            else if (d.y > d.z)
                return 1;
            else
                return 2;
        }

        //包围盒的面积
        double SurfaceArea() const
        {
            Vector3f d = Diagonal();
            return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
        }

        //计算包围盒中心点
        Vector3f Centroid()
        {
            return 0.5 * pMin + 0.5 * pMax;
        }

        //两个包围盒的相交：最小部分
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
                o.y /= pMax.y - pMin.y;
            if (pMax.z > pMin.z)
                o.z /= pMax.z - pMin.z;

            return o;
        }

        //判断两个包围盒是否有重叠部分
        bool Overlaps(const Bounds3& b1, const Bounds3& b2)
        {
            bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
            bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
            bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);

            return (x && y && z);
        }

        //判断点p是否在包围盒内
        bool Inside(const Vector3f& p, const Bounds3& b)
        {
            return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                    p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
        }

        //重载[]运算符：获得包围盒的最大值或最小值
        inline const Vector3f& operator[](int i) const
        {
            return (i == 0) ? pMin : pMax;
        }

        //射线是否与包围盒相交
        inline bool IntersectP(const Ray& ray, const Vector3f& invDir, const std::array<int, 3>& dirIsNeg) const;
};

//射线是否与包围盒相交
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir, const std::array<int, 3>& dirIsNeg) const
{
    //invDir: 射线方向(x, y, z)，invDir = (1.0/x, 1.0/y, 1.0/z)，因为使用乘法比除法更快
    //dirIsNeg: 射线方向(x, y, z)，dirIsNeg = [int(x>0), int(y>0), int(z>0)]，用这个来简化你的逻辑
    //待办事项：测试射线边界是否相交
    //进入时间和出去时间包围盒
    float t_min_x = (pMin.x - ray.origin.x) * invDir.x;
    float t_min_y = (pMin.y - ray.origin.y) * invDir.y;
    float t_min_z = (pMin.z - ray.origin.z) * invDir.z;

    float t_max_x = (pMax.x - ray.origin.x) * invDir.x;
    float t_max_y = (pMax.y - ray.origin.y) * invDir.y;
    float t_max_z = (pMax.z - ray.origin.z) * invDir.z;

    if (dirIsNeg[0]) std::swap(t_min_x, t_max_x);
    if (dirIsNeg[1]) std::swap(t_min_y, t_max_y);
    if (dirIsNeg[2]) std::swap(t_min_z, t_max_z);

    //最小时间的中最大时间，最大时间中的最小时间
    float t_min = std::max(t_min_x, std::max(t_min_y, t_min_z));
    float t_max = std::min(t_max_x, std::min(t_max_y, t_max_z));

    //这里是个大坑，必须改为<=，因为有很多bound可能是z维度是0，此时，tmin等于tmax
    if (t_max >= 0 && t_min <= t_max)
        return true;
    
    return false;
}

//两个包围盒的并集
inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);

    return ret;
}

//包围盒与点的并集
inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);

    return ret;
}

#endif //RAYTRACING_BOUNDS3_H
