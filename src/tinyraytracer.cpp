#define _USE_MATH_DEFINES
#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

using namespace std;

// 结果图片的尺寸
const int img_width = 1024;
const int img_height = 1024;

// 采样点数目
const int num_samples = 16;

// 墙体的尺寸
const int wall_x = 12; // 房间宽度 2 * wall_x
const int wall_y = 12; // 房间高度 2 * wall_y
const int wall_z = 36; // 房间深度 wall_z

// 点光源
struct Light
{
    Vec3f position; // 位置
    float intensity; // 光强

    Light(const Vec3f& p, const float& i) : position(p), intensity(i) {}
};

// 材质
struct Material
{
    float refractive_index; // 折射率
    Vec4f albedo; // diffuse, specular, 反射，折射比率
    Vec3f diffuse_color; // 漫反射颜色
    float specular_exponent; // 反光度

    float fuzziness; // Fuzziness parameter for the metallic material

    Material(const float& r, const Vec4f& a, const Vec3f& color, const float& spec, const float& fuzz)
        : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec), fuzziness(fuzz) {}
    Material() : refractive_index(1), albedo(1, 0, 0, 0), diffuse_color(), specular_exponent() {}
};


// 球体
struct Sphere {
    Vec3f center; // 中心
    float radius; // 半径
    Material material; //材料

    Sphere(const Vec3f& c, const float& r, const Material& m) : center(c), radius(r), material(m) {}

    bool ray_intersect(const Vec3f& orig, const Vec3f& dir, float& t0) const {
        //  点c在射线上的投影是pc，向量以v开头
        Vec3f vpc = center - orig;
        float c = vpc * dir; // dir标准化向量，模为1，得到pc到p点距离。
        float b_square = vpc * vpc - c * c; // 得到圆中心c到射线距离平方
        if (b_square > radius * radius) return false; //不相交，距离大于半径
        float thc = sqrt(radius * radius - b_square); // 计算相交距离的长度

        // 计算相交点，判断射线起点是否在圆内
        t0 = c - thc;
        float t1 = c + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;

        return true;
    }
};

// 在单位球内生成一个随机三维点，为模糊材质创建随机反射方向
Vec3f random_in_unit_sphere()
{
    float theta = 2.0f * M_PI * rand() / RAND_MAX;
    float phi = acos(1.0f - 2.0f * rand() / RAND_MAX);
    float x = sin(phi) * cos(theta);
    float y = sin(phi) * sin(theta);
    float z = cos(phi);
    return Vec3f(x, y, z);
}

// 反射计算
Vec3f reflect(const Vec3f& I, const Vec3f& N, const float& fuzziness)
{
    Vec3f reflected = I - N * 2.f * (I * N);
    return reflected + random_in_unit_sphere() * fuzziness;
}

// 折射计算，斯涅尔定律（Snell's Law）
Vec3f refract(const Vec3f& I, const Vec3f& N, const float& refractive_index)
{
    float cosi = -std::max(-1.f, std::min(1.f, I * N));
    float etai = 1;
    float etat = refractive_index;
    Vec3f n = N;
    if (cosi < 0)
    {
        cosi = -cosi;
        swap(etai, etat);
        n = -N;
    }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);

    return k < 0 ? Vec3f(0, 0, 0) : I * eta + n * (eta * cosi - sqrt(k));
}

// 判断光线是否与场景中的任何物体（球体、墙壁）相交，并计算交点、法向量和材质
bool scene_intersect(const Vec3f& orig, const Vec3f& dir, const vector<Sphere>& spheres, Vec3f& hit, Vec3f& N, Material& material)
{
    // 判断光线是否与球体相交
    float spheres_dist = numeric_limits<float>::max();
    for (size_t i = 0; i < spheres.size(); i++)
    {
        float dist_i;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist)
        {
            spheres_dist = dist_i;
            // 计算投影点
            hit = orig + dir * dist_i;
            // 计算圆中心与投影的向量
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }

    // 判断光线是否与墙壁相交
    // 地板
    float down_wall_dist = std::numeric_limits<float>::max();
    if (abs(dir.y) > 1e-3)
    {
        // 计算射线是否与平面相交
        // 这里平面的任意一点为(0, 0, 0)
        float d = -(orig.y + wall_y) / dir.y;
        Vec3f pt = orig + dir * d;

        // 限定平面y = -12 的大小长度和宽度
        if (d > 0 && abs(pt.x) < wall_x && pt.z < 0 && pt.z > -wall_z && d < spheres_dist)
        {
            down_wall_dist = d;
            hit = pt;
            N = Vec3f(0, 1, 0);
            material.diffuse_color = Vec3f(0.8, 0.8, 0.8);
            material.diffuse_color = material.diffuse_color * .3;
        }
    }

    // 天花板
    float up_wall_dist = std::numeric_limits<float>::max();
    if (abs(dir.y) > 1e-3)
    {
        float d = -(orig.y - wall_y) / dir.y;
        Vec3f pt = orig + dir * d;

        if (d > 0 && abs(pt.x) < wall_x && pt.z < 0 && pt.z > -wall_z && d < spheres_dist)
        {
            up_wall_dist = d;
            hit = pt;
            N = Vec3f(0, 1, 0);
            material.diffuse_color = Vec3f(1.0, 1.0, 1.0);
            material.diffuse_color = material.diffuse_color * .3;
        }
    }

    // 左墙
    float left_wall_dist = std::numeric_limits<float>::max();
    if (abs(dir.x) > 1e-3)
    {
        float d = -(orig.x + wall_x) / dir.x;
        Vec3f pt = orig + dir * d;

        if (d > 0 && abs(pt.y) < wall_y && pt.z < 0 && pt.z > -wall_z && d < spheres_dist)
        {
            left_wall_dist = d;
            hit = pt;
            N = Vec3f(0, 0, 1);
            material.diffuse_color = Vec3f(0.8, 0.3, 0.3);
            material.diffuse_color = material.diffuse_color * .3;
        }
    }

    // 右墙
    float right_wall_dist = std::numeric_limits<float>::max();
    if (abs(dir.x) > 1e-3)
    {
        float d = -(orig.x - wall_x) / dir.x;
        Vec3f pt = orig + dir * d;

        if (d > 0 && abs(pt.y) < wall_y && pt.z < 0 && pt.z > -wall_z && d < spheres_dist)
        {
            right_wall_dist = d;
            hit = pt;
            N = Vec3f(0, 0, 1);
            material.diffuse_color = Vec3f(0.3, 0.8, 0.3);
            material.diffuse_color = material.diffuse_color * .3;
        }
    }

    // 背墙
    float back_wall_dist = std::numeric_limits<float>::max();
    if (abs(dir.z) > 1e-3)
    {
        float d = -(orig.z + wall_z) / dir.z;
        Vec3f pt = orig + dir * d;

        if (d > 0 && abs(pt.x) < wall_x && abs(pt.y) < wall_y && d < spheres_dist)
        {
            back_wall_dist = d;
            hit = pt;
            N = Vec3f(0, 0, 1);
            material.diffuse_color = Vec3f(0.8, 0.8, 0.8);
            material.diffuse_color = material.diffuse_color * .3;
        }
    }

    if (spheres_dist < 1000 || up_wall_dist < 1000 || down_wall_dist < 1000 || left_wall_dist < 1000 || right_wall_dist < 1000 || back_wall_dist < 1000)
        return true;
    return false;
}

// 在场景中发射光线，考虑反射和折射，处理光照，计算交点的颜色
Vec3f cast_ray(const Vec3f& orig, const Vec3f& dir, const vector<Sphere>& spheres, const vector<Light>& lights, size_t depth = 0) {
    Vec3f point, N;
    Material material;

    if (depth > 4 || !scene_intersect(orig, dir, spheres, point, N, material))
    {
        return Vec3f(0.2, 0.2, 0.2); // 深色背景
    }

    // 计算反射和折射方向
    Vec3f reflect_dir = reflect(dir, N, material.fuzziness).normalize();
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    // 计算反射和折射起点
    Vec3f reflect_orig = reflect_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
    Vec3f refract_orig = refract_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
    // 迭代进行光追
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);

    // 计算diffuse和specular
    float diffuse_light_intensity = 0;
    float specular_light_intensity = 0;

    for (size_t i = 0; i < lights.size(); i++)
    {
        Vec3f light_dir = (lights[i].position - point).normalize();
        float light_distance = (lights[i].position - point).norm();

        Vec3f shadow_orig = light_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;

        // 沿着灯方向走，看看有没有物体相交
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt - shadow_orig).norm() < light_distance)
            continue;

        // 根据光照计算公式计算diffuse
        diffuse_light_intensity += max(0.f, light_dir * N) * lights[i].intensity;
        // 根据光照计算公式计算specular
        specular_light_intensity += pow(max(0.f, reflect(light_dir, N, material.fuzziness) * dir), material.specular_exponent) * lights[i].intensity;
    }

    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1.f, 1.f, 1.f) * specular_light_intensity * material.albedo[1]
        + reflect_color * material.albedo[2] + refract_color * material.albedo[3];
}

// 渲染场景
void render(const vector<Sphere>& spheres, vector<Light>& lights) {
    const int width = img_width;
    const int height = img_height;
    const int fov = M_PI / 3.;
    vector<Vec3f> framebuffer(width * height);

#pragma omp parallel for
    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            // 抗锯齿
            Vec3f color(0, 0, 0);

            // 多次采样
            int s = sqrt(num_samples);
            for (int si = 0; si < s; si++) {
                float dir_x = (i + (1.0 * si) / s) - width / 2.;
                for (int sj = 0; sj < s; sj++) {
                    float dir_y = -(j + (1.0 * sj) / s) + height / 2.; // 反转图像
                    float dir_z = -height / (2.0 * tan(fov / 2.0));
                    color = color + cast_ray(Vec3f(0, 0, 0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
                }
            }

            color = color * (1.0 / num_samples); // 取平均值

            framebuffer[i + j * width] = color;
        }
        if (j % 10 == 0)
            cout << j << "/" << height << endl;
    }

    vector<unsigned char> pixmap(width * height * 3);
    for (size_t i = 0; i < width * height; i++)
    {
        Vec3f& c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max > 1) c = c * (1. / max);
        for (size_t j = 0; j < 3; j++)
        {
            pixmap[i * 3 + j] = (unsigned char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }

    stbi_write_jpg("out.jpg", width, height, 3, pixmap.data(), 100);
}

int main(int argc, char** argv) {
    // 定义球体材料
    Material  light_blue_pearl(1.0, Vec4f(0.6, 0.3, 0.1, 0.0), Vec3f(0.6, 0.8, 1.0), 50.,0.0); // 浅蓝色光泽球体
    Material      glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8), 125., 0.0); // 玻璃
    Material      glossy_metal(1.0, Vec4f(0.9, 0.1, 0.9, 0.0), Vec3f(0.7, 0.7, 0.7), 10.0, 0.0); // 光泽金属
    Material fuzzy_metal(1.0, Vec4f(0.9, 0.1, 0.9, 0.0), Vec3f(0.7, 0.7, 0.7), 10.0, 0.6); // Fuzzy metallic material
                                                                                            //Material      forsted_metal(1.0, Vec4f(0.9, 0.9, 0.9, 0.0), Vec3f(0.7, 0.7, 0.7), 10.0); // 磨砂金属
    Material     mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425., 0.0); // 镜子
    Material yellow_rubber(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.4, 0.4, 0.3), 10., 0.0); //乳白色橡胶材质

    // 在场景中添加球体
    vector<Sphere> spheres;
    spheres.push_back(Sphere(Vec3f(0, -10, -23), 2.5, light_blue_pearl));
    spheres.push_back(Sphere(Vec3f(-6.5, 7, -27), 2, glossy_metal));
    spheres.push_back(Sphere(Vec3f(-2, 7, -27), 2, fuzzy_metal));
    spheres.push_back(Sphere(Vec3f(2, 4, -27), 3, glass));
    spheres.push_back(Sphere(Vec3f(6, -8, -27), 3, mirror));
    spheres.push_back(Sphere(Vec3f(-5, -8, -27), 4, yellow_rubber));

    // 在场景中添加光源
    vector<Light> lights;
    lights.push_back(Light(Vec3f(0, 11.8, -20), 2.0f));

    // 渲染场景
    render(spheres, lights);

    return 0;
}