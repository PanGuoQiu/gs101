#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

//材质类型：散射的
enum MaterialType { DIFFUSE };

//材质类
class Material
{
    private:
        //技术反射方向
        Vector3f reflect(const Vector3f& I, const Vector3f& N) const
        {
            return I - 2 * dotProduct(I, N) * N;
        }

        //使用斯涅尔定律计算反射方向
        //我们需要小心处理两种可能的情况：
        //  当射线在对象里面
        //  当射线在对象外面
        //如果射线是在外面，你需要让cosi为正cosi = -N·I
        //如果射线是在里面，你需要翻转折射率和求法线N的负数
        Vector3f refract(const Vector3f& I, const Vector3f& N, const float& ior) const
        {
            float cosi = clamp(-1, 1, dotProduct(I, N));
            float etai = 1, etat = ior;
            Vector3f n = N;
            if (cosi < 0)
                cosi = -cosi;
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
        //参数I是入射视图方向
        //参数N是相交点法线
        //参数ior是材质折射率
        //参数[输出]kr是反射光的量
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

        //转换为世界坐标
        Vector3f toWorld(const Vector3f& a, const Vector3f& N)
        {
            Vector3f B, C;
            if (std::fabs(N.x) > std::fabs(N.y))
            {
                float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
                C = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);                
            }
            else
            {
                float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
                C = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
            }

            B = crossProduct(C, N);

            return a.x * B + a.y * C + a.z * N;
        }

    public:
        MaterialType m_type;                                    //材质类型
        //Vector3f m_color;                                     //材质颜色
        Vector3f m_emission;                                    //自发光
        float ior;                                              //折射率
        Vector3f Kd, Ks;                                        //散射系数、镜面系数
        float specularExponent;                                 //镜面指数，控制高光的大小
        //Texture tex;                                          //纹理
        
        //构造函数
        inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));

        inline MaterialType getType();                          //获得材质的类型
        //inline Vector3f getColor();                           //获得材质的颜色
        inline Vector3f getColorAt(double u, double v);         //得到u、v处的颜色
        inline Vector3f getEmission();                          //获得自发光
        inline bool hasEmission();                              //是否有自发光

        //通过材质属性对射线进行采样
        inline Vector3f sample(const Vector3f& wi, const Vector3f& N);
        //获得一条射线，计算这条射线的PdF
        inline float pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
        //获得一条射线，计算这条射线的贡献
        inline Vector3f eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
};

//构造函数
Material::Material(MaterialType t, Vector3f e)
{
    m_type = t;
    //m_color = c;
    m_emission = e;
}

//获得材质类型
MaterialType Material::getType()
{
    return m_type;
}


//Vector3f Material::getColor()
//{
//    return m_color;
//}

//获得自发光位置
Vector3f Material::getEmission()
{
    return m_emission;
}

//是否有自发光
bool Material::hasEmission()
{
    if (m_emission.norm() > EPSILON)
        return true;
    else
        return false;
}

//获得uv处的颜色
Vector3f Material::getColorAt(double u, double v)
{
    return Vector3f();
}

//通过材质属性对射线进行采样
Vector3f Material::sample(const Vector3f& wi, const Vector3f& N)
{
    switch (m_type)
    {
        case DIFFUSE:
        {
            //半球上的均匀采样
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
            return toWorld(localRay, N);

            break;
        }
    }
}

//获得一条射线，计算这条射线的PdF
float Material::pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
{
    switch (m_type)
    {
        case DIFFUSE:
        {
            //统一样本概率 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;

            break;
        }
    }
}

//获得一条射线，计算这条射线的贡献
Vector3f Material::eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
{
    switch (m_type)
    {
        case DIFFUSE:
        {
            //计算散射模型的贡献
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f)
            {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);

            break;
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
