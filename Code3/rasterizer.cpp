#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

//载入顶点位置
rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return { id };
}

//载入索引值
rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return { id };
}

//载入颜色值
rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return { id };
}

//载入法线
rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;                                             //当前法线的索引值

    return { id };
}

//Bresenham的线段绘制算法
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    //获得线段起点和终点
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    //绘制线段的颜色
    Eigen::Vector3f line_color = { 255, 255, 255};              //白色

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx = x2 - x1;                                               //dx为x的增量
    dy = y2 - y1;                                               //dy为y的增量
    dx1 = fabs(dx);                                             //dx1为dx的绝对值
    dy1 = fabs(dy);                                             //dy1为dy的绝对值
    px = 2 * dy1 - dx1;                                         //判别式：px < 0，则y不变，否则y + 1
    py = 2 * dx1 - dy1;                                         //判别式：py < 0，则x不变，否则x + 1

    //判断线段的斜率的绝对值是否小于等于1
    if (dy1 <= dx1)
    {
        //判断是否需要交换线段的两个端点
        if (dx >= 0)
        {
            //x的增量大于0，则不需要
            x = x1;
            y = y1;
            xe = x2;

        }
        else
        {
            //x的增量小于0，则需要
            x = x2;
            y = y2;
            xe = x1;
        }

        //设置(x,y)像素的颜色
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        //绘制线段
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

            //设置像素的颜色
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
        }
    }
    else
    {
        //判断是否需要交换线段的两个端点
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

        //设置(x,y)像素的颜色
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        //迭代线段的每个像素
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

            //设置像素的颜色
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
        }
    }
}

//将3维向量转换为4维向量
auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//判断一个点是否在三角形内
static bool insideTriangle(int x, int y, const Vector4f* _v)
{
    //存储三角形的三个顶点
    Vector3f v[3];
    for (int i = 0; i < 3; i++)
        v[i] = { _v[i].x(), _v[i].y(), 1.0f};
    
    //使用叉乘的结果判断点是否在三角形内
    Vector3f f0, f1, f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);

    Vector3f p(x, y, 1.0);
    if ((p.dot(f0) * f0.dot(v[2]) > 0) && (p.dot(f1) * f1.dot(v[0]) > 0) && (p.dot(f2) * f2.dot(v[1]) > 0))
        return true;
    
    return false;
}

//2维重心坐标的计算
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    
    return std::make_tuple(c1,c2,c3);
}

//绘制
void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList)
{
    //深度值
    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.2;

    //计算MVP
    Eigen::Matrix4f mvp = projection * view * model;
    //遍历模型的每个三角形
    for (const auto& t : TriangleList)
    {
        Triangle newtri = *t;

        //计算三角形三个顶点在视图矩阵中的坐标
        std::array<Eigen::Vector4f, 3> mm
        {
            (view * model * t->v[0]), 
            (view * model * t->v[1]), 
            (view * model * t->v[2])
        };

        //视图空间位置是顶点的摄像机坐标
        //newtri是投影后的三角形
        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        //投影后的三角形：标准设备坐标空间
        Eigen::Vector4f v[] = 
        {
            mvp * t->v[0], 
            mvp * t->v[1], 
            mvp * t->v[2]
        };

        //除以齐次坐标
        for (auto& vec : v)
        {
            vec /= vec.w();
        }

        //逆矩阵的转置
        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = 
        {
            inv_trans * to_vec4(t->normal[0], 0.0f), 
            inv_trans * to_vec4(t->normal[1], 0.0f), 
            inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        //视口变换
        for (auto& vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            //屏幕空间坐标
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //视图空间法线
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148, 121.0, 92.0);
        newtri.setColor(1, 148, 121.0, 92.0);
        newtri.setColor(2, 148, 121.0, 92.0);

        //总是通过视图空间顶点位置
        rasterize_triangle(newtri, viewspace_pos);
    }
}

//计算3D三角形的重心坐标作为坐标插值
static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

//计算2D平面的重心坐标作为纹理插值
static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

//光栅化屏幕空间(光栅化三角形)
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos)
{
    auto v = t.toVector4();
    
    //待办事项：查找当前三角形的外部包围盒
    //查找AABB
    float max_x = -1e6, max_y = -1e6;
    float min_x = 1e6, min_y = 1e6;
    //遍历三角形的三个顶点
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

    //向下取整和向上取整
    int min_x_i = std::floor(min_x);
    int max_x_i = std::ceil(max_x);
    int min_y_i = std::floor(min_y);
    int max_y_i = std::ceil(max_y);

    //迭代像素并判断当前像素是否在三角形内
    for (int x = min_x_i; x < width && x <= max_x_i; ++x)
    {
        for (int y = min_y_i; y < height && y <= max_y_i; ++y)
        {
            if (insideTriangle(x, y, &v[0]))
            {
                float alpha, beta, gamma;
                std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, t.v);
                
                //通过插值得到z的值
                float z_interpolate = alpha * v[0].z() + beta * v[1].z() + gamma * v[2].z();
                float weight_sum = alpha + beta + gamma;
                z_interpolate /= weight_sum;

                //获得当前像素点的索引值，并更新深度缓存和像素的各个属性
                int id = get_index(x, y);
                if (z_interpolate < depth_buf[id])
                {
                    //更新深度缓存
                    depth_buf[id] = z_interpolate;

                    //通过插值计算像素的颜色、法线、纹理坐标和着色坐标
                    //颜色
                    Eigen::Vector3f interpolated_color = (alpha * t.color[0] + beta * t.color[1] + gamma * t.color[2]) / weight_sum;

                    //法线
                    Eigen::Vector3f interpolated_normal = (alpha * t.normal[0] + beta * t.normal[1] + gamma * t.normal[2]) / weight_sum;
                    interpolated_normal.normalize();

                    //纹理坐标
                    Eigen::Vector2f interpolated_tex_coor = (alpha * t.tex_coords[0] + beta * t.tex_coords[1] + gamma * t.tex_coords[2]) / weight_sum;

                    //着色坐标
                    Eigen::Vector3f interpolated_shadingcoords = (alpha * view_pos[0] + beta * view_pos[1] + gamma * view_pos[2]) / weight_sum;

                    //设置像素的颜色和法线
                    fragment_shader_payload payload(interpolated_color, interpolated_normal, interpolated_tex_coor, texture ? &*texture : nullptr);
                    payload.view_pos = interpolated_shadingcoords;

                    //设置像素颜色
                    set_pixel(Eigen::Vector2i(x, y), fragment_shader(payload));
                }
            }
        }
    }

    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;
    
    // interpolate the attributes of fragments in triangle t

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);
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

//清空缓存区域
void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        //把帧缓冲区域填充为0
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }

    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        //把深度缓冲区域填充为 最大值
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

//构造函数
rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    //计算帧缓存区域和深度缓存区域的大小
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    //
    texture = std::experimental::nullopt;
}

//获得(x,y)的索引值
int rst::rasterizer::get_index(int x, int y)
{
    return (height - y) * width + x;
}

//设置(x,y)处像素点的颜色值
void rst::rasterizer::set_pixel(const Vector2i& point, const Eigen::Vector3f& color)
{
    //旧索引：auto ind = point.y() + point.x() * width;
    int ind = (height - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

//设置顶点着色器
void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

//设置片段着色器
void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}
