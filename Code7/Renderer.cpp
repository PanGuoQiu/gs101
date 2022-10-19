#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include "omp.h"

//角度转换为弧度
inline float deg2rad(const float& deg)
{
    return deg * M_PI / 180.0;
}

const float EPSILON = 0.00001;

//主渲染函数，这里我们迭代图像中所有的像素
//生成主射线并把这些光线投射到场景中。framebuffer的内容被保存到一个文件中
void Renderer::Render(const Scene& scene, int spp)
{
    //帧缓冲区的大小
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));                    //垂直视角
    float imageAspectRatio = scene.width / (float)scene.height;     //宽高比
    Vector3f eye_pos(278, 273, -800);                               //眼睛的位置

    //改变spp变量值来改变样品量
    std::cout<<"SPP: "<<spp<<"\n";
    for (int j = 0; j < scene.height; ++j)
    {
        omp_set_num_threads(8);
        #pragma omp parallel for
        for (int i = 0; i < scene.width; ++i)
        {
            //generate primary ray direction
            int m = j * scene.width + i;
            float x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            //sampled rays in the same pixel
            for (int k = 0; k < spp; k++)
            {
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
            }
        }

        UpdateProgress(j / (float)scene.height);
    }

    UpdateProgress(1.f);

    //把帧缓冲区中的数据存储到.ppm格式的文件中
    FILE* fp = fopen(("binary_" + std::to_string(spp) + ".ppm").c_str(), "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }

    fclose(fp);
}
