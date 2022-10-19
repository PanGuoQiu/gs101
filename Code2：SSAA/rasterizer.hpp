#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include "global.hpp"
#include "Triangle.hpp"
using namespace Eigen;

//声明自己的命名空间rst
namespace rst
{
    //枚举缓存类型
    enum class Buffers
    {
        Color = 1, 
        Depth = 2
    };

    //重载|(或)运算符
    inline Buffers operator|(Buffers a, Buffers b)
    {
        return Buffers((int)a | (int)b);
    }

    //重载&(与)运算符
    inline Buffers operator&(Buffers a, Buffers b)
    {
        return Buffers((int)a & (int)b);
    }

    //枚举原始图元类型
    enum class Primitive
    {
        Line, 
        Triangle
    };

    //为了满足好奇：编写的函数带有两个缓存编号是作为函数的参数，
    //下面的两个结构体是确保如果你混淆它们的命令，编译器就不能编译它。又名：类型安全

    //顶点位置缓存编号结构体
    struct pos_buf_id
    {
        int pos_id = 0;
    };

    //顶点索引缓存编号结构体
    struct ind_buf_id
    {
        int ind_id = 0;
    };

    //顶点颜色缓存编号结构体
    struct col_buf_id
    {
        int col_id = 0;
    };

    //光栅化类
    class rasterizer
    {
        public:
            rasterizer(int w, int h);                                                   //构造函数

            pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);   //载入顶点位置
            ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);       //载入顶点索引
            col_buf_id load_colors(const std::vector<Eigen::Vector3f>& colors);         //载入顶点颜色

            //设置MVP矩阵
            void set_model(const Eigen::Matrix4f& m);                                   //设置模型矩阵
            void set_view(const Eigen::Matrix4f& v);                                    //设置视图矩阵
            void set_projection(const Eigen::Matrix4f& p);                              //设置投影矩阵

            //设置像素点的颜色值
            void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);

            //清除缓存区域
            void clear(Buffers buff);

            //绘制，根据参数：顶点缓存区域编号、索引缓存区域编号、颜色缓存区域编号，基本图元类型
            void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);

            //获得帧缓冲
            std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

            private:
                void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);             //绘制原始图元：直线
                void rasterize_triangle(const Triangle& t);                            //光栅化三角形

                //图形管线
                //Vertex Shader -> MVP -> Clipping -> /.W -> Viewport -> DrawLine/DrawTri -> Fragment Shader

            private:
                Eigen::Matrix4f model;                                                  //模型矩阵
                Eigen::Matrix4f view;                                                   //视图矩阵
                Eigen::Matrix4f projection;                                             //投影矩阵

                std::map<int, std::vector<Eigen::Vector3f>> pos_buf;                    //顶点缓存
                std::map<int, std::vector<Eigen::Vector3i>> ind_buf;                    //索引缓存
                std::map<int, std::vector<Eigen::Vector3f>> col_buf;                    //颜色缓存

                std::vector<Eigen::Vector3f> frame_buf;                                 //帧缓存
                std::vector<float> depth_buf;                                           //深度缓存

                //int get_index(int x, int y);                                            //获得(x,y)处的像素点
                int get_index(int x, int y, int i, int j);

                //各个缓存区域的大小
                int width, height;

                int next_id = 0;                                                        //初始化编号为0
                int get_next_id() { return next_id++; }                                 //获得下一个编号的值
    };
}
