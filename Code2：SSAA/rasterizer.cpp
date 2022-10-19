#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

//载入顶点位置
rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

//载入顶点索引
rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

//载入顶点颜色
rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

//3维向量转换为4维向量
auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//判断像素点是否在三角形内
static bool insideTriangle(float x, float y, const Vector3f* _v)
{
     //作业
    //待办事项：执行这个函数并检查点(x,y)是否在代表点_v[0]、_v[1]、_v[2]三角形内
    Eigen::Vector3f p(x, y, 0);
    Eigen::Vector3f v0(_v[0].x(), _v[0].y(), 0), v1(_v[1].x(), _v[1].y(), 0), v2(_v[2].x(), _v[2].y(), 0);

    bool t1, t2, t3;
    //使用叉乘计算，并判断点是否在三角形内
    t1 = ((v1 - v0).cross(p - v0))(2) > 0;
    t2 = ((v2 - v1).cross(p - v1))(2) > 0;
    t3 = ((v0 - v2).cross(p - v2))(2) > 0;

    return ((t1 == t2) && (t1 == t3));
}

//计算三角形的重心中标(2D)，并返回三个系数
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    
    return std::make_tuple(c1,c2,c3);
}

//光栅化绘制
void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    //计算MVP矩阵
    Eigen::Matrix4f mvp = projection * view * model;
    //遍历顶点索引
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f), 
            mvp * to_vec4(buf[i[1]], 1.0f), 
            mvp * to_vec4(buf[i[2]], 1.0f)
        };

        //除以齐次坐标
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

        //设置顶点位置
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        //获得顶点颜色
        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        //设置顶点颜色
        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        //光栅化三角形
        rasterize_triangle(t);
    }
}

//使用SSAA光栅化三角形
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
    auto v = t.toVector4();
    
    //作业
    //待办事项：寻找当前三角形的外包围盒
    //寻找AABB包围盒
    float max_x = -1e6, max_y = -1e6;
    float min_x = 1e6, min_y = 1e6;

    for (int i = 0; i != 3; ++i)
    {
        if (v[i](0) < min_x)
            min_x = v[i](0);
        if (v[i](0) > max_x)
            max_x = v[i](0);

        if (v[i](1) < min_y)
            min_y = v[i](1);
        if (v[i](1) > max_y)
            max_y = v[i](1);
    }

    //floor:向下取整，ceil:取大于或等于的最小整数
    int min_x_i = std::floor(min_x);
    int max_x_i = std::ceil(max_x);
    int min_y_i = std::floor(min_y);
    int max_y_i = std::ceil(max_y);

    //待办事项：寻找当前三角形的外包包围盒
    //迭代像素并判断当前像素是否在三角形内
    for (int x = min_x_i; x <= max_x_i; x++)
    {
        for (int y = min_y_i; y <= max_y_i; y++)
        {
            float d[] = {0.0f, 0.5f};
            int inside_num = 0;
            Eigen::Vector3f sample_list = {0, 0, 0};
            for (int i =0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    float x1 = x + d[i];
                    float y1 = y + d[j];
                    
                    //判断像素点是否在三角形内
                    if (insideTriangle(x1 + 0.25f, y1 + 0.25f, t.v))
                    {
                        sample_list += t.getColor();
                        //如果这样，使用更随的代码获得插值z的值，即计算(x,y)处的深度值
                        auto [alpha, beta, gamma] = computeBarycentric2D(x1 + 0.25f, y1 + 0.25f, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        //待办事项：设置当前像素(使用set_pixel函数)的三角形的颜色(使用getColor函数)如果三角形需要绘制。
                        int index = get_index(x, y, i, j);
                        if (z_interpolated < depth_buf[index])
                        {
                            //更新缓冲区中的值
                            depth_buf[index] = z_interpolated;
                            inside_num++;
                        }
                    }
                }
            }

            if (inside_num > 0)
                set_pixel(Vector3f(x, y, 1), sample_list / 4);

        }
    }
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

//清除缓存区域
void rst::rasterizer::clear(rst::Buffers buff)
{
    //填充帧缓冲区域
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f(0, 0, 0));
    }

    //填充深度缓冲区域
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

//构造函数，缓存区域宽和高扩大2倍
rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    //面积扩大4倍
    frame_buf.resize(4 * w * h);
    depth_buf.resize(4 * w * h);
}

//获得(x,y)的像素点中的(i,j)
int rst::rasterizer::get_index(int x, int y, int i, int j)
{
    int nPixel = (height - 1 - y) * width + x;                  //正常缓冲区域的像素点索引值
    //计算扩大的缓冲区域的像素点索引值
    return 4 * nPixel + 2 * j + i;
}

//设置(x,y)的像素
void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //旧的索引：auto ind = point.y() + point.x() * width;
    //计算像素点，和设置像素颜色
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}
