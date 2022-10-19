#pragma once

#include <Eigen/Eigen>
#include <experimental/optional>
#include <algorithm>
#include "global.hpp"
#include "Shader.hpp"
#include "Triangle.hpp"

using namespace Eigen;

//rst命名空间
namespace rst
{
    //枚举缓存类型
    enum class Buffers
    {
        Color = 1, 
        Depth = 2
    };

    //缓存类型的重载|运算符
    inline Buffers operator|(Buffers a, Buffers b)
    {
        return Buffers((int)a | (int)b);
    }

    //缓存类型的重载&运算符
    inline Buffers operator&(Buffers a, Buffers b)
    {
        return Buffers((int)a & (int)b);
    }

    //枚举原始图元
    enum class Primitive
    {
        Line, 
        Triangle
    };

    //为了满足好奇：编写的函数带有两个缓存编号是作为函数的参数，
    //下面的两个结构体是确保如果你混淆它们的命令，编译器就不能编译它。又名：类型安全

    //顶点缓存的索引值
    struct pos_buf_id
    {
        int pos_id = 0;
    };

    //索引缓存的索引值
    struct ind_buf_id 
    {
        int ind_id = 0;
    };

    //颜色缓存的索引值
    struct col_buf_id
    {
        int col_id = 0;
    };

    //光栅化类
    class rasterizer
    {
        public:
            //构造函数
            rasterizer(int w, int h);

            pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);   //
            ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);       //
            col_buf_id load_colors(const std::vector<Eigen::Vector3f>& colors);         //
            col_buf_id load_normals(const std::vector<Eigen::Vector3f>& normals);       //

            //设置MVP矩阵
            void set_model(const Eigen::Matrix4f& m);                                   //模型矩阵
            void set_view(const Eigen::Matrix4f& v);                                    //视图矩阵
            void set_projection(const Eigen::Matrix4f& p);                              //投影矩阵

            //设置纹理
            void set_texture(Texture tex) { texture = tex; }

            //设置顶点着色器和片段着色器
            void set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader);
            void set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader);

            //设置像素(x,y)的颜色
            void set_pixel(const Vector2i& point, const Eigen::Vector3f& color);

            //清空缓存
            void clear(Buffers buff);

            //绘制
            void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);
            void draw(std::vector<Triangle *> &TriangleList);

            //返回帧缓存
            std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }
        
        private:
            //绘制线段
            void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);

            //光栅化三角形 参数：三角形类、三角形三个顶点所在世界坐标
            void rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& world_pos);

            //渲染管线
            // VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI -> FRAGSHADER
        
        private:
            Eigen::Matrix4f model;                                                      //模型矩阵
            Eigen::Matrix4f view;                                                       //视图矩阵
            Eigen::Matrix4f projection;                                                 //投影矩阵

            //法线编号
            int normal_id = -1;

            //顶点各个属性的缓存区域
            std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
            std::map<int, std::vector<Eigen::Vector3i>> ind_buf;
            std::map<int, std::vector<Eigen::Vector3f>> col_buf;
            std::map<int, std::vector<Eigen::Vector3f>> nor_buf;

            //
            std::experimental::optional<Texture> texture;

            //存储方式
            std::function<Eigen::Vector3f(fragment_shader_payload)> fragment_shader;
            std::function<Eigen::Vector3f(vertex_shader_payload)> vertex_shader;

            //帧缓存和深度缓存
            std::vector<Eigen::Vector3f> frame_buf;
            std::vector<float> depth_buf;

            //获得(x,y)处像素点的编号
            int get_index(int x, int y);

            //缓存的宽和高
            int width, height;

            //顶点当前编号和下一个编号
            int next_id = 0;
            int get_next_id() { return next_id++; }
    };
}
