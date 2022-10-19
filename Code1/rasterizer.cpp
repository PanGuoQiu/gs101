#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>

//位置缓冲区域：载入位置
rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    //获得索引id，并插入位置值
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

//索引缓冲区域：载入索引值
rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
    //获得索引id，并插入索引值
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

//Bresenham的线段绘制算法
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    //获得线段的起点和终点坐标
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    //绘制线段的颜色
    Eigen::Vector3f line_color = {255, 255, 255};

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx = x2 - x1;                                               //x增量
    dy = y2 - y1;                                               //y增量
    dx1 = fabs(dx);                                             //x增量的绝对值
    dy1 = fabs(dy);                                             //y增量的绝对值
    px = 2 * dy1 - dx1;                                         //判别式：px<0，则y不变，否则y+1
    py = 2 * dx1 - dy1;                                         //判别式：py<0，则x不变，否则x+1

    //判断斜率是否小于等于1
    if (dy1 <= dx1)
    {
        //判断线段两端的坐标是否要交换
        if (dx >= 0)
        {
            x = x1;
            y = y1;
            xe = x2;
        }
        else
        {
            x = x2;
            y = y2;
            xe = x1;
        }

        //设置(x,y)像素的颜色
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point, line_color);
        for (i = 0; x < xe; i++)
        {
            x = x + 1;
            if (px < 0)
            {
                px = px + 2 * dy1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    y = y + 1;
                }
                else
                {
                    y = y - 1;
                }

                px = px + 2 * (dy1 - dx1);
            }

            //设置(x,y)像素的颜色
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, line_color);
        }
    }
    else
    {
        if (dy >= 0)
        {
            x = x1;
            y = y1;
            ye = y2;
        }
        else
        {
            x = x2;
            y = y2;
            ye = y1;
        }

        //设置(x, y)的像素颜色
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point, line_color);
        for (i = 0; y < ye; i++)
        {
            y = y + 1;
            if (py <= 0)
            {
                py = py + 2 * dx1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    x = x + 1;
                }
                else
                {
                    x = x - 1;
                }

                py = py + 2 * (dx1 - dy1);
            }

            //设置(x,y)像素的颜色
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, line_color);
        }
    }
}

//3维向量转换为4维向量
auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//绘制，参数：位置缓存、索引缓存、基础图元
void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type)
{
    //判断基础图元
    if (type != rst::Primitive::Triangle)
    {
        throw std::runtime_error("绘制图元然而除了三角形还没生成");
    }

    auto buf = pos_buf[pos_buffer.pos_id];                      //位置缓冲区：获得vector数组的开始位置编号
    auto ind = ind_buf[ind_buffer.ind_id];                      //索引缓冲区：获得vector数组的开始位置编号

    float f1 = (100 - 0.1) / 2.0;
    float f2 = (100 + 0.1) / 2.0;

    //计算从3维空间变换到2维空间的矩阵变换
    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;

        //把三角形每个顶点位置都变换到2维屏幕中
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f), 
            mvp * to_vec4(buf[i[1]], 1.0f), 
            mvp * to_vec4(buf[i[2]], 1.0f)
        };

        //齐次坐标为1.0，在同一个平面内
        for (auto& vec : v)
        {
            vec /= vec.w();
        }

        //视口变换
        for (auto& vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        //设置三角形三个顶点的位置
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        //设置三角形三个点的颜色
        t.setColor(0, 255.0, 0.0, 0.0);
        t.setColor(1, 0.0, 255.0, 0.0);
        t.setColor(2, 0.0, 0.0, 255.0);
        //绘制三角形线框
        rasterize_wireframe(t);
    }
}

//绘制三角形线框
void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
    //绘制三角形的三条边
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());
}

//设置模型矩阵
void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

//设置视图矩阵
void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

//设置投影矩阵
void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

//清空缓存
void rst::rasterizer::clear(rst::Buffers buff)
{
    //判断使用哪个缓存，并清除
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        //如果是颜色缓存，则在帧缓存中填充0
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }

    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        //如果是深度缓存，则在深度缓存中填充“无穷大的值”
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

//设置缓冲区域的大小
rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);                                    //重置帧缓存区域的大小
    depth_buf.resize(w * h);                                    //重置深度缓存区域的大小
}

//获得(x,y)的索引值
int rst::rasterizer::get_index(int x, int y)
{
    return (height - y) * width + x;                            //为什么是(height - y)？因为左下角为(0, 0)
}

//设置像素颜色
void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //旧的索引：auto ind = point.y() + point.x() * width;
    //判断点是否在窗口中
    if (point.x() < 0 || point.x() >= width || point.y() < 0 || point.y() >= height)
        return;
    
    //计算帧缓中每个像素的颜色
    auto ind = (height - point.y()) * width + point.x();
    frame_buf[ind] = color;
}
