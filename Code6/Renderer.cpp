#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"


//角度转弧度
inline float deg2rad(const float& deg)
{
    return deg * M_PI / 180.0;
}

//误差值
const float EPSILON = 0.00001;

// 主渲染函数，这里我们迭代图像中所有的像素
// 生成主射线并把这些光线投射到场景中。framebuffer的内容被保存到一个文件中
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);              //帧缓冲的大小

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio =scene.width / (float)scene.height;                  //宽高比
    Vector3f eye_pos(-1, 5, 10);                                                //眼睛的位置
    int m;
    for (uint32_t j = 0; j < scene.height; ++j)
    {
        for (uint32_t i = 0; i < scene.width; ++i)
        {
            //生成主射线方向
            float x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
            //摄像机在看哪个方向？
            //默认方向与世界坐标相同
            
            //不要忘记归一化这个方向
            Vector3f dir = normalize(Vector3f(x, y, -1));
            Ray ray(eye_pos, dir);
            framebuffer[m++] = scene.castRay(ray, 0);
            // = (eye_pos, dir, scene, 0);
        }

        //更新进度
        UpdateProgress(j / (float)scene.height);
    }

    //更新进度
    UpdateProgress(1.f);

    //保存帧缓存的数据到文件
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }

    fclose(fp);
}
