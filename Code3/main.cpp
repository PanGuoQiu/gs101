#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

//视图矩阵
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

//模型矩阵，Y轴旋转
Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.0f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0, 
                -sin(angle), 0, cos(angle), 0, 
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0, 
             0, 2.5, 0, 0, 
             0, 0, 2.5, 0, 
             0, 0, 0, 1;
    
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0, 
                 0, 1, 0, 0, 
                 0, 0, 1, 0, 
                 0, 0, 0, 1;

    return translate * rotation * scale;
}

//投影矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    //待办事项：使用一样的投影矩阵来自显示参数
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float top = tan(eye_fov / 2) * zNear;
    float right = aspect_ratio * top;

    //获得投影矩阵中的正交矩阵
    Eigen::Matrix4f ortho_p = Eigen::Matrix4f::Identity();
    ortho_p << 1/right, 0, 0, 0, 
               0, 1/top, 0, 0, 
               0, 0, 2/(zFar - zNear), (zNear + zFar) / 2.0, 
               0, 0, 0, 1;
    
    //获得挤压矩阵
    Eigen::Matrix4f squeezing = Eigen::Matrix4f::Identity();
    squeezing << -zNear, 0, 0, 0, 
                 0, -zNear, 0, 0, 
                 0, 0, -(zNear + zFar), zNear * zFar, 
                 0, 0, 1, 0;
    
    projection = ortho_p * squeezing;

    return projection;
}

//顶点着色器
Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

//片段着色器
Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.0f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

//反射
static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

//光照结构体
struct light
{
    Eigen::Vector3f position;               //位置
    Eigen::Vector3f intensity;              //强度
};

//纹理着色器
Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    //获得纹理坐标中的颜色
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        //待办事项：获得在当前片段着色器的纹理坐标的纹理值
        return_color = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]);
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    //光照系数
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);          //环境光系数
    Eigen::Vector3f kb = texture_color / 255.0f;                        //散射光系数
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);       //镜面光系数

    //光源
    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};                               //光源数组
    Eigen::Vector3f amb_light_instensity{10, 10, 10};                   //环境光强度
    Eigen::Vector3f eye_pos{0, 0, 10};                                  //视点位置

    float p = 150;                                                      //指数，控制强光范围

    //像素的属性
    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};                           //最终的颜色
    Eigen::Vector3f to_eye = eye_pos - point;                           //像素点指向视点的方向
    to_eye.normalize();

    for (auto& light : lights)
    {
        //待办事项：遍历光源的代码，计算环境光、散射光和镜面光，并组合。然后，计算结果并保存在result_color中
        Eigen::Vector3f l = light.position - point;
        double squared_r = l.squaredNorm();

        l.normalize();
        Eigen::Vector3f h = (to_eye + l).normalized();
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_instensity);
        Eigen::Vector3f diffuse = kb.cwiseProduct(light.intensity / squared_r * std::fmax(0.0f, normal.dot(l)));
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity / squared_r * std::pow(std::fmax(0.0f, normal.dot(h)), p));

        return_color += ambient + diffuse + specular;
    }

    return result_color * 255.0f;
}

//phong着色模型
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    //光照系数
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    //光源
    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};                               //光源数组
    Eigen::Vector3f amb_light_intensity{10, 10, 10};                    //环境光强度
    Eigen::Vector3f eye_pos{0, 0, 10};                                  //摄像机位置

    float p = 150;                                                      //指数，控制强光范围

    //像素属性
    Eigen::Vector3f color = payload.color;                              //像素的颜色
    Eigen::Vector3f point = payload.view_pos;                           //视点位置
    Eigen::Vector3f normal = payload.normal;                            //法线

    Eigen::Vector3f result_color = {0, 0, 0};                           //最终颜色
    Eigen::Vector3f to_eye = eye_pos - point;                           //摄像机的聚焦方向
    to_eye.normalize();

    //遍历光源
    for (auto& light : lights)
    {
        //遍历每个光源代码，计算环境光、散射光和镜面光的组合。然后，计算最终的结果在result_colr中。
        Eigen::Vector3f l = light.position - point;
        double squared_r = l.squaredNorm();

        l.normalize();
        Eigen::Vector3f h = (to_eye + l).normalized();
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity / squared_r * std::fmax(0.0f, normal.dot(l)));
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity / squared_r * std::pow(std::fmax(0.0f, normal.dot(h)), p));

        result_color += ambient + diffuse + specular;
    }

    return result_color * 255.0f;
}

//位移映射
Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    //光照系数
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    //光源
    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};                               //光源数组
    Eigen::Vector3f amb_light_intensity{10, 10, 10};                    //环境光强度
    Eigen::Vector3f eye_pos{0, 0, 10};                                  //视点位置

    float p = 150;                                                      //指数，控制强光范围

    Eigen::Vector3f color = payload.color;                              //片段着色器颜色
    Eigen::Vector3f point = payload.view_pos;                           //视点
    Eigen::Vector3f normal = payload.normal;                            //法线

    //计算移动点的高度
    float huv = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]).norm();
    float hu1v = payload.texture->getColor(payload.tex_coords[0] + 1.0f / payload.texture->width, payload.tex_coords[1]).norm();
    float huv1 = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1] + 1.0f / payload.texture->height).norm();

    //缩放
    float kh = 0.2, kn = 0.1;

    //移动点，并计算新的法线
    point += payload.view_pos + kn * normal * huv;
    double du = kh * kn * (hu1v - huv);
    double dv = kh * kn * (huv1 - huv);
    Eigen::Vector3f new_normal(-du, -dv, 1.0f);

    float x = normal(0), y = normal(1), z = normal(2);
    //使辅助轴heip.dot(normal) = 0
    Eigen::Vector3f help_axis_0(x * y / sqrt(x * x + z * z), -sqrt(x * x + z * z), z * y / sqrt(x * x + z * z));
    help_axis_0.normalize();
    Eigen::Vector3f help_axis_1 = normal.cross(help_axis_0);
    help_axis_1.normalize();

    //计算TBN矩阵
    Eigen::Matrix3f T;
    T.block<3, 1>(0, 2) = normal;
    T.block<3, 1>(0, 0) = help_axis_0;
    T.block<3, 1>(0, 1) = help_axis_1;
    normal = T * new_normal;
    normal.normalize();

    //怎加phong模型
    Eigen::Vector3f result_color = {0, 0, 0};
    Eigen::Vector3f to_eye = eye_pos - point;
    to_eye.normalize();

    for (auto& light : lights)
    {
        //待办事项：对于代码中的每个光源，计算环境光、散射光和镜面光，并组合。然后，计算结果保存在result_color对象中.
        Eigen::Vector3f l = light.position - point;                     //视点指向光源
        double squared_r = l.squaredNorm();                             //光线的平方

        l.normalize();
        Eigen::Vector3f h = (to_eye + l).normalized();                  //半角向量
        //环境光、散射光和镜面光
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity / squared_r * std::fmax(0.0f, normal.dot(l)));
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity / squared_r * std::pow(std::fmax(0.0f, normal.dot(h)), p));

        //结果
        result_color += ambient + diffuse + specular;
    }

    //返回结果
    return result_color * 255.0f;
}

//凹凸贴图或法线贴图
Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    //光照系数
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    //光源
    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    //设置其他自定变量
    std::vector<light> lights = {l1, l2};                                   //光源数组
    Eigen::Vector3f amb_light_intensity{10, 10, 10};                        //环境光强度
    Eigen::Vector3f eye_pos{0, 0, 10};                                      //眼睛的位置

    //像素的属性
    Eigen::Vector3f color = payload.color;                                  //颜色
    Eigen::Vector3f point = payload.view_pos;                               //视点
    Eigen::Vector3f normal = payload.normal;                                //法线

    //应增加边界检测以避免分割故障
    float huv = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]).norm();
    float hu1v = payload.texture->getColor(payload.tex_coords[0] + 1.0f / payload.texture->width, payload.tex_coords[1]).norm();
    float huv1 = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1] + 1.0f / payload.texture->height).norm();

    float kh = 0.2, kn = 0.1;

    //计算新的法线
    double du = kh * kn * (hu1v - huv);
    double dv = kh * kn * (huv1 - huv);
    Eigen::Vector3f new_normal(-du, -dv, 1.0f);

    float x = normal(0), y = normal(1), z = normal(2);
    //确保 辅助轴help.dot(normal) = 0
    Eigen::Vector3f help_axis_0(x * y/ sqrt(x * x + z * z), -sqrt(x * x + z * z), z * y / sqrt(x * x + z * z));
    Eigen::Vector3f help_axis_1 = normal.cross(help_axis_0);

    Eigen::Matrix3f T;
    T.block<3, 1>(0, 2) = normal;
    T.block<3, 1>(0, 0) = help_axis_0;
    T.block<3, 1>(0, 1) = help_axis_1;
    
    normal = T * new_normal;
    normal.normalize();

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]

    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    // bump mapping

    // firstly, we need to transform the b

    Eigen::Vector3f result_color ={0, 0, 0};
    result_color = normal;

    return result_color * 255.0f;
}

//主函数
int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    //载入.obj文件
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for (auto mesh : Loader.LoadedMeshes)
    {
        for (int i = 0; i < mesh.Vertices.size(); i += 3)
        {
            Triangle* t = new Triangle();
            for (int j = 0; j < 3; j++)
            {
                t->setVertex(j, Vector4f(mesh.Vertices[i+j].Position.X, mesh.Vertices[i+j].Position.Y, mesh.Vertices[i+j].Position.Z, 1.0));
                t->setNormal(j, Vector3f(mesh.Vertices[i+j].Normal.X, mesh.Vertices[i+j].Normal.Y, mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j, Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }

            TriangleList.push_back(t);
        }
    }

    //光栅化窗口的大小
    rst::rasterizer r(700, 700);

    //纹理
    auto texture_path = "hmap.jpg";                         //凹凸纹理
    //auto texture_path = "spot_texture.png";                 //正常纹理
    r.set_texture(Texture(obj_path + texture_path));        //纹理路径

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")                     //正常纹理
        {
            std::cout<<"Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")                 //法线贴图
        {
            std::cout<<"Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")                  //phong模型
        {
            std::cout<<"Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")                   //凹凸贴图
        {
            std::cout<<"Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")           //位移贴图
        {
            std::cout<<"Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0, 0, 10};

    //顶点着色和片段着色
    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename + ".png", image);

        return 0;
    }

    //按键处理
    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename + ".png", image);
        key = cv::waitKey(key);

        if (key == 'a')
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }
    }

    return 0;
}
/*

#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float top = tan(eye_fov / 2) * zNear;
    float right = aspect_ratio * top; 
    // get orthographic project matrix
    Eigen::Matrix4f ortho_p =  Eigen::Matrix4f::Identity();
    ortho_p << 1 / right, 0, 0, 0,
                0, 1 / top, 0, 0,
                0, 0, 2 / (zFar - zNear),  (zNear + zFar) / 2.0,
                0, 0, 0, 1;
    // get the squeezing matrix

    Eigen::Matrix4f squeezing =  Eigen::Matrix4f::Identity();
    squeezing << -zNear, 0, 0, 0,
                0, -zNear, 0, 0,
                0, 0, -(zNear + zFar), zNear * zFar,
                0, 0, 1, 0;
    
    projection = ortho_p * squeezing;
    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]);
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    Eigen::Vector3f to_eye = eye_pos - point; 
    to_eye.normalize();

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f l = light.position - point;
        double squared_r =  l.squaredNorm();
        
        l.normalize();
        Eigen::Vector3f h = (to_eye + l).normalized();
        Eigen::Vector3f ambient = ka.cwiseProduct( amb_light_intensity);
        Eigen::Vector3f diffuse = kd.cwiseProduct( light.intensity / squared_r * std::fmax(0.0f, normal.dot(l))) ;
        Eigen::Vector3f specular = ks.cwiseProduct( light.intensity / squared_r * std::pow(std::fmax(0.0f, normal.dot(h)), p) ) ;
        result_color += ambient + diffuse + specular;
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;
    
    Eigen::Vector3f result_color = {0, 0, 0};
    Eigen::Vector3f to_eye = eye_pos - point; 
    to_eye.normalize();
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f l = light.position - point;
        double squared_r = l.squaredNorm();
        
        l.normalize();
        Eigen::Vector3f h = (to_eye + l).normalized();
        Eigen::Vector3f ambient = ka.cwiseProduct( amb_light_intensity);
        Eigen::Vector3f diffuse = kd.cwiseProduct( light.intensity / squared_r * std::fmax(0.0f, normal.dot(l))) ;
        Eigen::Vector3f specular = ks.cwiseProduct( light.intensity / squared_r * std::pow(std::fmax(0.0f, normal.dot(h)), p) ) ;
        result_color += ambient + diffuse + specular;
    }
    // std::cout<<result_color.transpose()<<std::endl;
    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float huv = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]).norm();
    float hu1v = payload.texture->getColor(payload.tex_coords[0] + 1.0f / payload.texture->width, payload.tex_coords[1]).norm();
    float huv1 = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1] + 1.0f / payload.texture->height).norm();
    


    float kh = 0.2, kn = 0.1;
    
    // move the point
    point += payload.view_pos +  kn * normal * huv;
    double du = kh * kn * (hu1v-huv);
    double dv = kh * kn * (huv1-huv);
    Eigen::Vector3f new_normal(-du, -dv, 1.0);

    float x = normal(0), y = normal(1), z = normal(2);
    // to make help .dot (normal) = 0
    Eigen::Vector3f help_axis_0(x * y / sqrt(x*x + z * z), - sqrt(x * x + z * z), z * y / sqrt(x * x + z * z) );
    help_axis_0.normalize();
    Eigen::Vector3f help_axis_1 = normal.cross(help_axis_0);
    help_axis_1.normalize();
    Eigen::Matrix3f T;
    T.block<3, 1>(0, 2) = normal;
    T.block<3, 1>(0, 0) = help_axis_0;
    T.block<3, 1>(0, 1) = help_axis_1;
    normal = T * new_normal;
    normal.normalize();
    // add phong
    Eigen::Vector3f result_color = {0, 0, 0};
    Eigen::Vector3f to_eye = eye_pos - point; 
    to_eye.normalize();
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f l = light.position - point;
        double squared_r = l.squaredNorm();
        
        l.normalize();
        Eigen::Vector3f h = (to_eye + l).normalized();
        Eigen::Vector3f ambient = ka.cwiseProduct( amb_light_intensity);
        Eigen::Vector3f diffuse = kd.cwiseProduct( light.intensity / squared_r * std::fmax(0.0f, normal.dot(l))) ;
        Eigen::Vector3f specular = ks.cwiseProduct( light.intensity / squared_r * std::pow(std::fmax(0.0f, normal.dot(h)), p) ) ;
        result_color += ambient + diffuse + specular;
    }
    // result_color = normal;
    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;
    // should add border check to avoid segmentation fault.
    float huv = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]).norm();
    float hu1v = payload.texture->getColor(payload.tex_coords[0] + 1.0f / payload.texture->width, payload.tex_coords[1]).norm();
    float huv1 = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1] + 1.0f / payload.texture->height).norm();
    
    float kh = 0.2, kn = 0.1;
    
    double du = kh * kn * (hu1v-huv);
    double dv = kh * kn * (huv1-huv);
    Eigen::Vector3f new_normal(-du, -dv, 1.0);

    float x = normal(0), y = normal(1), z = normal(2);
    // to make help .dot (normal) = 0
    Eigen::Vector3f help_axis_0(x * y / sqrt(x*x + z * z), - sqrt(x * x + z * z), z * y / sqrt(x * x + z * z) );
    Eigen::Vector3f help_axis_1 = normal.cross(help_axis_0);
    
    Eigen::Matrix3f T;
    T.block<3, 1>(0, 2) = normal;
    T.block<3, 1>(0, 0) = help_axis_0;
    T.block<3, 1>(0, 1) = help_axis_1;
    normal = T * new_normal;
    normal.normalize();
    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]

    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    // bump mapping

    // firstly, we need to transform the b
    Eigen::Vector3f result_color = {0, 0, 0};

    result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    // auto texture_path = "spot_texture.png";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";

            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename + ".png", image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename + ".png", image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
*/