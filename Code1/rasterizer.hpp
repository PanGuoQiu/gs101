#pragma once

#include "Triangle.hpp"
#include <algorithm>
#include <Eigen/Eigen>
using namespace Eigen;

namespace rst
{
    //缓存类型
    enum class Buffers
    {
        Color = 1, 
        Depth = 2
    };

    //重载|（或）运算符
    inline Buffers operator|(Buffers a, Buffers b)
    {
        return Buffers((int)a | (int)b);
    }

    //重载&(与)运算符
    inline Buffers operator&(Buffers a, Buffers b)
    {
        return Buffers((int)a & (int)b);
    }

    //原始图元类型
    enum class Primitive
    {
        Line, 
        Triangle
    };

    //为了满足好奇：编写的函数带有两个缓存编号是作为函数的参数，
    //下面的两个结构体是确保如果你混淆它们的命令，编译器就不能编译它。又名：类型安全
    
    //位置缓存编号结构体
    struct pos_buf_id
    {
        int pos_id = 0;
    };

    //索引缓存编号结构体
    struct ind_buf_id 
    {
        int ind_id = 0;
    };

    //光栅化器类
    class rasterizer
    {
        public:
            //构造函数
            rasterizer(int w, int h);                                   //设置窗口的大小

            //载入顶点的位置和索引值
            pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positioins);
            ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);

            void set_model(const Eigen::Matrix4f& m);                   //设置模型矩阵
            void set_view(const Eigen::Matrix4f& v);                    //设置视图矩阵
            void set_projection(const Eigen::Matrix4f& p);              //设置投影矩阵

            //设置像素
            void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);

            //清除缓存
            void clear(Buffers buff);

            //绘制
            void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, Primitive type);

            //帧缓存
            std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

        private:
            void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end); //绘制线段
            void rasterize_wireframe(const Triangle& t);                //绘制三角形线框

        private:
            Eigen::Matrix4f model;                                      //模型矩阵
            Eigen::Matrix4f view;                                       //视图矩阵
            Eigen::Matrix4f projection;                                 //投影矩阵

            std::map<int, std::vector<Eigen::Vector3f>> pos_buf;        //存储每个顶点的编号和位置
            std::map<int, std::vector<Eigen::Vector3i>> ind_buf;        //存储每个顶点的编号和索引

            std::vector<Eigen::Vector3f> frame_buf;                     //帧缓存（颜色缓存区域）
            std::vector<float> depth_buf;                               //深度缓存（区域）
            int get_index(int x, int y);                                //获得(x, y)的索引值

            int width, height;                                          //窗口的宽度和高度

            int next_id = 0;                                            //初始化下一个编号为0
            int get_next_id() { return next_id++; }                     //获得下一个编号
    };

}   //namespace rst
