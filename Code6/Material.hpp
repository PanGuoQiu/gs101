#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

//材质类型
enum MaterialType { DIFFUSE_AND_GLOSSY, REFLECTION_AND_REFRACTION, REFLECTION };

//材质类
class Material
{
    public:
        MaterialType m_type;                                    //材质类型
        Vector3f m_color;                                       //材质颜色
        Vector3f m_emission;                                    //自发光
        float ior;                                              //折射率
        float Kd, Ks;                                           //散射系数、镜面系数
        float specularExponent;                                 //镜面指数
        //Texture tex;                                          //纹理

        //构造函数
        inline Material(MaterialType t = DIFFUSE_AND_GLOSSY, Vector3f c = Vector3f(1, 1, 1), Vector3f e = Vector3f(0, 0, 0));

        inline MaterialType getType();                          //获得材质类型
        inline Vector3f getColor();                             //获得材质颜色
        inline Vector3f getColorAt(double u, double v);         //获得纹理颜色
        inline Vector3f getEmission();                          //获得自发光

};

//构造函数
Material::Material(MaterialType t, Vector3f c, Vector3f e)
{
    m_type = t;
    m_color = c;
    m_emission = e;
}

//获得材质类型
MaterialType Material::getType()
{
    return m_type;
}

//获得材质颜色
Vector3f Material::getColor()
{
    return m_color;
}

//获得自发光
Vector3f Material::getEmission()
{
    return m_emission;
}

//获得纹理颜色
Vector3f Material::getColorAt(double u, double v)
{
    return Vector3f();
}

#endif //RAYTRACING_MATERIAL_H
