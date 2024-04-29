# 光线追踪

## 一、综述

Whitted-style 光线跟踪方法是指从视点向成像平面上的像素发射光线，找到与该光线相交的最近物体的交点，如果该点处的表面是散射面，则计算光源直接照射该点产生的颜色；如果该点处表面是镜面或折射面，则继续向反射或折射方向跟踪另一条光线，如此递归下去，直到光线逃逸出场景或达到设定的最大递归深度。

### 光线追踪算法原理

图形学中的光线的性质：

 沿直线传播

 光线和光线间无法碰撞

 光线路径可逆

光线追踪主要遵循第三条性质光线可逆。

第一步：Ray Casting：从相机向投影的近平面上的每个像素发射一条光线，判断和场景中的物体的交点(有多个交点取最近的交点)。再连接交点和光源，判断该连线间是否有物体，若有说明该交点在阴影中。利用Blinn-Phong模型对该点进行局部光照模型计算，得到该像素的颜色，该颜色就是目前像素点的颜色。

<img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=YmNiZTFhYWU0MmQ3YTQyZWMyZjM4ZDQ0M2IwZjQwMmFfczgwa21qU3VMbVJ1WFR6S3g5Y09Dd2g1clRyTmYxd0RfVG9rZW46WEpIWGJwOE15b0FCUmp4ZjF6b2MzNDllblFmXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom: 80%;" />

第二步：Whitted-Style（Recursive） Ray Tracing：只考虑第一步Ray Casting，仍和和局部光照模型差不多，我们还需考虑全局效果。光线和物体相交会发生反射与折射，反射与折射的光线和其他物体相交继续发生反射与折射，将这些交点和光线相连,对这些点都计算颜色，最后将这些颜色全部按照权重累加，最终得到近平面该像素点的颜色。

<img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=MGJiNTQ0MTkwOWMyOGFjMDlmOWI4YWVkYzJkOTQyNzhfa01wUDJzNHhQcWtzbXJmaEJock5WaklOU2dkTEdZUTRfVG9rZW46SmFpTGIxVmZJb1d4ZkF4S2FabGNydnJXbjhaXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom:80%;" />

## 二、实现流程

### 1、添加球体

```C++
// 球体
struct Sphere {
    Vec3f center; // 中心
    float radius; // 半径
    Material material; //材料

    Sphere(const Vec3f& c, const float& r, const Material& m) : center(c), radius(r), material(m) {}

    bool ray_intersect(const Vec3f& orig, const Vec3f& dir, float& t0) const {
        Vec3f vpc = center - orig; //圆心和起点的向量
        float c = vpc * dir; // dir是当前像素和观察点的向量的标准化向量，模为1（也就是射线）
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
```

定义球体类，球体类中的`ray_intersect`用于判断对于每个像素点所形成的一条来自摄像机并穿过该像素点的射线是否与球体相交，是否相交的判断进一步通过计算球心与当前像素射线的距离得到，并且根据这一距离可以求得射线的起点是否在圆内，从而得到位于射线正方向上交点距离。

对于如何得到一条来自摄像机并穿过该像素点的射线，是根据屏幕的长宽比与相机视场角（设为60度）将像素平面的点转换为世界中的三维射线向量，计算得到射线点的x、y、z坐标，进而得到射线向量。

<img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=ZTM3ZWM3MGNmOGMyNzVlZDg1NDMxN2ZiMTdhN2M1ZWVfN0FMVlpWc3pMTmlwdk1hMUdTWHd3OXk0Yk90STRCOUVfVG9rZW46TVFuemJEUkZKbzNMUmd4OURUOWMzV3JqbjdCXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom:50%;" />

```C++
float dir_x = (i + 0.5) - width / 2.;
float dir_y = -(j + 0.5) + height / 2.;    // this flips the image at the same time
float dir_z = -height / (2. * tan(fov / 2.));
Vec3f dir = Vec3f(dir_x, dir_y, dir_z).normalize();
```

当我们需要添加多个球体时，就只需要保留距离摄像机更近那个交点的像素颜色。

```C++
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
```

#### 球体材质

```C++
struct Material
{
    float refractive_index; // 折射率
    Vec4f albedo; // 漫反射, 镜面反射, 反射，折射比率
    Vec3f diffuse_color; // 漫反射颜色
    float specular_exponent; // 反光度

    float fuzziness; // Fuzziness parameter for the metallic material

    Material(const float& r, const Vec4f& a, const Vec3f& color, const float& spec, const float& fuzz)
        : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec), fuzziness(fuzz) {}
    Material() : refractive_index(1), albedo(1, 0, 0, 0), diffuse_color(), specular_exponent() {}
};
```

球体材质需要包括球体折射率、albedo参数、漫反射颜色反光度与磨砂程度。

1. `refractive_index`（折射率）：
   1. 描述了材质对光线的折射程度。折射率越高，光线在材质中的弯曲程度就越大。
2. `albedo`（漫反射、镜面反射、反射、折射比率）：
   1. 是一个四维向量，表示材质对不同光照成分的反应。具体来说：
      - `albedo[0]`：漫反射比率，描述了光线经过漫反射后的强度。
      - `albedo[1]`：镜面反射比率，描述了光线经过镜面反射后的强度。
      - `albedo[2]`：反射比率，描述了光线经过反射后的强度。
      - `albedo[3]`：折射比率，描述了光线经过折射后的强度。
3. `diffuse_color`（漫反射颜色）：
   1. 描述了材质的漫反射颜色。漫反射是由于光线在表面微观结构上的散射而产生的，这个颜色反映了物体在漫反射光照下的外观。
4. `specular_exponent`（反光度）：
   1. 描述了镜面反射的光滑程度。这个值越高，反射光的集中度越高，表面越光滑。
5. `fuzziness`：
   1. 描述了材质的模糊程度，主要用于模拟非理想反射。这个参数用于模拟不完美的反射，使得反射光线的方向有一些随机性，增加了物体的模糊度与磨砂度。

我们定义了以下6种材料：

```C++
// 定义球体材料
Material  light_blue_pearl(1.0, Vec4f(0.6, 0.3, 0.1, 0.0), Vec3f(0.6, 0.8, 1.0),   50., 0.0); // 浅蓝色光泽球体
Material             glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8),  125., 0.0); // 玻璃
Material      glossy_metal(1.0, Vec4f(0.9, 0.1, 0.9, 0.0), Vec3f(0.7, 0.7, 0.7),  10.0, 0.0); // 光泽金属
Material       fuzzy_metal(1.0, Vec4f(0.9, 0.1, 0.9, 0.0), Vec3f(0.7, 0.7, 0.7),  10.0, 0.6); // 磨砂金属
Material            mirror(1.0, Vec4f(0.0, 10., 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425., 0.0); // 镜子
Material     yellow_rubber(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.4, 0.4, 0.3),   10., 0.0); // 乳白色橡胶材质
```

### 2、添加墙体

我们设置了一个方形盒子，左墙为红色墙，右墙为绿色墙。墙体渲染通过判断射线是否与墙体相交来完成，当射线位于给定的墙体范围大小内时，设置该点的交点向量、法向量以及漫反射颜色。在`scene_intersect`函数中，具体实现如下：

```C++
// 判断光线是否与墙壁相交
// 地板
float down_wall_dist = std::numeric_limits<float>::max();
if (abs(dir.y) > 1e-3)
{
    // 计算射线是否与平面相交
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
```

添加墙体前后对比（这里是已经实现了光追的图片，光追实现见后文）：

<img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=MTFkMDFmN2Y1Y2RmM2NkMGM4NGY5YWFiY2E2YTUxMTNfVzVTVHd6eUd6OFg2OGRlWVhHa1ZkU1hJYnprQU1zaEJfVG9rZW46WVlPUGJKNXJZb0RDZUl4YkZpTWMyY2YzbkZlXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom:50%;" /><img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=N2Y0MmY3ZWFiYTYwMTE1YmRmZGI5MDliOTJmMGVjNjBfR2tQMW45NUlGSDNBRnEyU3QzRVpKeVFyQm5yTmlrUUJfVG9rZW46RzdoRGJCbDBEb0xmb2l4bHNFQWNkQWI0bkFoXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom:50%;" />

### 3、反射与折射基本原理

根据反射公式，使用入射向量加上两倍的其在法向方向上的投影即为出射向量。

<img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=ZmRhNTU5NDllMzkxY2FhMzExMjkzMDc4MmVjMmViMjdfOUdrNU90SEdwanFGcTZKSkk0VEFVN3dkUWxSNktmeWdfVG9rZW46UWZQbGJzdVNEb1F4Zjd4NVVCTmNJSnFEblhjXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom:50%;" />

```C++
// 反射计算
Vec3f reflect(const Vec3f& I, const Vec3f& N, const float& fuzziness)
{
    Vec3f reflected = I - N * 2.f * (I * N);
    return reflected + random_in_unit_sphere() * fuzziness;
}
```

斯涅尔定律，也被称为折射定律，描述了光波在穿过折射率不同的两种介质时改变方向的规律。这个定律可以用数学方式表示为：

<img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=MWNjOGMzZDAzNzNkYjZiM2QwYWM4NzQ1ZTRhMmY5MDNfWmFoOTg5Wk1oWjZvMmZyWlB0OUs1V0Z2akRYemc3WnpfVG9rZW46UVpjRWJRYkowb0s2WFh4cko3SGNZUDY5bm5nXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom: 67%;" />

在这个方程中：

- *η* 和 *η*′ 分别表示两种介质的折射率。
- *θ* 表示入射角（入射光线与表面法线的夹角）。
- *θ*′ 表示折射角（折射光线与表面法线的夹角）。

<img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=NDBlY2I2ZDdiOTFkNWZiOTE5N2U4ZDc1OGQzMWI1MmNfNGRiTDI0U3dKR0dzTEZ4cm40RExJM01UaHRqRWF6NU1fVG9rZW46QnNxdmJBa0Q4b1dMNk14U3lMOGM0MHl6bjBmXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom:50%;" />

斯涅耳定律运用到出射向量的计算上，需要经过一系列数学推导，将出射向量表示为已知量的形式。

这里主要参考了http://t.csdnimg.cn/WgFKH这一链接中的推导过程，最后所得结果如下，并将其转化为代码形式。

<img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=NWQyNTNiMWFhZmQzMzkyZjQzYThhYTU5MWU0ZWVlZjFfMUhBQ0c4TGIwaVg0ZE15MTN4T0hFRWRFTWVaWW9rM3dfVG9rZW46T0gzSGJMQ3kzb0ZFNW14d0VCZGNGeU9mbkdmXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom:80%;" />

```C++
Vec3f refract(const Vec3f& I, const Vec3f& N, const float& refractive_index){
    // 计算入射角的余弦值
    float cosi = -std::max(-1.f, std::min(1.f, I * N));

    // 定义两个介质的折射指数
    float etai = 1;             // 空气或真空的折射指数（来自空气）
    float etat = refractive_index; // 目标介质的折射指数

    Vec3f n = N; // 法线向量，默认指向折射的目标介质

    // 如果入射角小于0，表示光线从目标介质中出射，则需调整法线方向
    if (cosi < 0)
    {
        cosi = -cosi;
        swap(etai, etat);
        n = -N; // 反转法线方向
    }

    // 计算折射比率
    float eta = etai / etat;
    // 描述了描述了折射的情况
    float k = 1 - eta * eta * (1 - cosi * cosi);

    // 如果k小于0，表示发生全反射，光线不穿透，返回黑色（Vec3f(0, 0, 0)）
    // 在光线从高折射指数的介质射向低折射指数的介质时发生
    if (k < 0)
        return Vec3f(0, 0, 0);
    else
    {
        // 计算折射光线的方向
        return I * eta + n * (eta * cosi - sqrt(k));
    }
}
```

### 4、添加光源

定义一个光源结构体，使用点光源，包括光源位置、光强信息。

```C++
// 点光源
struct Light
{
    Vec3f position; // 位置
    float intensity; // 光强

    Light(const Vec3f& p, const float& i) : position(p), intensity(i) {}
};
```

我们需要对光源发出的光进行漫反射与镜面光处理。

#### 漫反射

光照原理：

- 漫反射光照是一种光线与表面法线之间的夹角越大，光照强度越弱的光照效果。这是由于光线在表面上散射的结果。表面法线是垂直于表面的向量，而光线与法线之间的夹角越小，表面对光的反射就越强烈。

在实现时，循环遍历场景中的所有光源，计算光线与法线夹角的余弦值（取其与0的较大值，确保漫反射光照的强度不为负数），然后乘以光照强度以得到漫反射强度。

```C++
diffuse_light_intensity += max(0.f, light_dir * N) * lights[i].intensity;
```

具体解释：

- `light_dir` 是光源方向的单位向量。
- `N` 是表面法向量的单位向量。
- `light_dir * N` 计算光源方向与法向量的点积。点积表示两个向量之间的夹角关系，即夹角的余弦值，范围在[-1, 1]之间。当两个向量指向相同方向时，点积为1，指向相反方向时，点积为-1，垂直时为0。
- `max(0.f, light_dir * N)` 用于确保漫反射光照的强度不会为负数。如果光源方向与法向量夹角过大，导致点积小于零，将其截断为零。
- `lights[i].intensity` 是光源的强度。
- `max(0.f, light_dir * N) * lights[i].intensity`得到漫反射强度

#### 镜面光

光照原理：

- 镜面光照也决定于光的方向向量和物体的法向量，但是它也决定于观察方向，例如玩家是从什么方向看向这个片段的。
- 镜面光照决定于表面的反射特性。
- 根据法向量翻折入射光的方向来计算反射向量。然后我们计算反射向量与观察方向的角度差，它们之间夹角越小，镜面光的作用就越大。由此产生的效果就是，我们看向在入射光在表面的反射方向时，会看到一点高光。

得到视线方向向量与沿着法线轴的反射向量后，我们就可以计算出镜面分量。镜面分量的计算方法为：视线方向与反射方向的点乘（并确保它不是负值），然后取它的“反光度”次幂。

```C++
specular_light_intensity += pow(max(0.f, reflect(light_dir, N, material.fuzziness) * dir), material.specular_exponent) * lights[i].intensity;
```

具体解释：

- `light_dir` 是光源方向的单位向量。
- `N` 是表面法向量的单位向量。
- `material.fuzziness` 参数用于控制镜面反射的模糊程度。
- `dir` 是视线方向单位向量。
- `material.specular_exponent`是高光的反光度。
- `lights[i].intensity`是光源的强度。
- `reflect(light_dir, N, material.fuzziness)`：这部分调用了我们前面实现的 `reflect` 函数，返回了反射光方向的单位向量。
- `reflect(light_dir, N, material.fuzziness) * dir`：计算了反射光线与视线方向 `dir` 之间的点积。这个点积反映了观察者视角对反射光线的投影，用于确定反射光线的方向与观察者视线的夹角。
- `max(0.f, ...)`：确保夹角余弦值不为负数，因为镜面反射的光照在不同的表面部分可能为负，此时光照不会对最终的颜色产生影响。
- `pow(..., material.specular_exponent)`：使用镜面指数将夹角余弦值进行幂运算。这个指数控制了镜面高光的锐利度，值越大，高光越锐利。
- `pow(...) * lights[i].intensity`：最后，乘以光源的强度，以计算出镜面反射的光照强度。

### 5、迭代光追

基本原理如下：

- 通过递归地追踪光线，考虑反射和折射，可以模拟光线在场景中的传播和反射，从而计算出每个像素的颜色。
- 当光线与物体相交时，根据材质属性计算反射和折射的方向，并继续追踪这两条光线，直到达到递归深度上限或不再与物体相交。
- 最终，通过合成漫反射、镜面高光以及递归追踪的颜色，得到场景中每个像素的最终颜色。

```C++
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

    [...计算光线漫反射、镜面反射强度...]
    
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1.f, 1.f, 1.f) * specular_light_intensity * material.albedo[1]
        + reflect_color * material.albedo[2] + refract_color * material.albedo[3];
}
```

具体解释：

1. 参数解释：
   1. `orig`：光线的起点，初始值为摄像机位置。
   2. `dir`：光线的方向，初始值为摄像机指向的点。
   3. `spheres`：场景中的物体，这里是球体的集合。
   4. `lights`：光源的集合。
   5. `depth`：递归追踪的深度，默认为0。
2. 初始化变量：
   1. `point`：交点的位置。
   2. `N`：表面法向量。
   3. `material`：材质信息。
3. 判断递归深度和光线是否与场景中的物体相交：
   1. 如果递归深度超过4次（`depth > 4`）或者光线未与任何物体相交，返回深色背景颜色 `Vec3f(0.2, 0.2, 0.2)`。
4. 计算反射和折射的方向和起点：
   1. 使用 `reflect` 函数计算反射光线的方向，考虑了材质的模糊度。
   2. 使用 `refract` 函数计算折射光线的方向。
5. 进行光线追踪的递归：
   1. 调用 `cast_ray` 函数追踪反射光线和折射光线，得到它们的颜色。
6. 计算光照强度：
   1. 初始化漫反射和镜面反射的光照强度。
   2. 对于每个光源：
      - 计算光源方向和距离。
      - 判断是否有阴影，即沿着光源方向走，检查是否有物体阻挡光线到达表面。
      - 计算漫反射和镜面反射的光照强度，分别加到对应的变量上。
7. 计算最终颜色：
   1. 使用计算得到的漫反射和镜面反射的光照强度，结合材质的属性，计算最终的颜色。
   2. 返回的颜色由漫反射、镜面反射、反射光线和折射光线的颜色组成。

### 6、抗锯齿

通过对一个像素点的多次采样取颜色的平均值来实现抗锯齿。我们设置了一个`num_samples`为采样次数，这里我们选取的采样次数为16，即讲一个像素点划分为16个方格，在每个方格内进行采样；对于每个采样点，进行光线追踪获取对应的颜色后将这些颜色累积到 `color` 变量中；最后对所有采样得到的颜色进行平均，以获得最终像素的颜色。

原来的代码：

```C++
float dir_x = (i + 0.5) - width / 2.;
float dir_y = -(j + 0.5) + height / 2.;
float dir_z = -height / (2.0 * tan(fov / 2.0));
framebuffer[i + j * width] = cast_ray(Vec3f(0, 0, 0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
```

实现抗锯齿的代码：

```C++
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
```

实现抗锯齿前后对比：

<img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=NmY1MTAwOTA4ZDJjMThiYjVhNGMyNWMzOTg3YzU3MDVfS01RclJwUVFCTEg0UzZkWnBkSDdUQ0E1a0MyMFVvdnZfVG9rZW46R3hjdmJGUkthbzlmYXJ4dHZaMGNBZ3BvblNjXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom:50%;" /><img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=MzNiNWJlN2U0ZTNkZjc0MTQ3NzBkZTFjZmQ5ODY1YTNfS3dMTWdWQmN3bHc3Nk9zaDlVZ0d6bTlXZUdKQ0pVcndfVG9rZW46T1VDeWI1N0Jvb291dTJ4T2VyZWNsVjdPbndjXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom:50%;" />

局部细节前后对比：

<img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=NzZlZDA1NmEwZTdmZjNmY2QxZWU2ZTAwZWNmZDE4ZTlfWTc1aWJZVTFtNXNiZDFQNDNSZWFCYVJudXRUQmpvYTdfVG9rZW46RXRVRGJEM1lBbzlDVUF4eGRMaGNiUVhoblFoXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom:50%;" /><img src="https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=MjEzMGJhMmNjYjI2MjVjMzc1MmNkNzI1MDFkMGExODRfMkJiZ0tKRTVVcFFneW14MzVsaGlDdWRJNFdSa0tJYXJfVG9rZW46V29YbGJNaVJUbzBlSjF4eHJqRGMxM2tUbjVnXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA" alt="img" style="zoom:50%;" />

### 7、渲染画面

使用OpenMP多线程渲染，提高渲染速度。

将光线追踪得到的颜色值结果`framebuffer`转换为用于可视化的图像数据`pixmap`，其中颜色值被映射到范围 [0, 255]。最后，通过stb_image 库中的`stbi_write_jpg` 函数，将图像数据写入 JPEG 文件中。

```C++
    // OpenMP多线程渲染
    #pragma omp parallel for
    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
        // 获取颜色
        [...framebuffer[i + j * width] = color;...]
        }
    }
    
    // 转换输出
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
```

## 三、结果展示

![img](https://dan23ole6lp.feishu.cn/space/api/box/stream/download/asynccode/?code=OGZlYTM4MjQ0MjdkOGM3ODQ0MDc4MjFmZWRmM2EwZmRfd01Ca3NESmMya3B2UXloSTdUdkw5ZlpRT25zSlRFWmRfVG9rZW46S0FOeGJiUndRb1FWek54NjRqRWNkTUkxbkNnXzE3MTQzODE5OTM6MTcxNDM4NTU5M19WNA)