
#import "../appx/theme.typ": tufte, note, mermaid
#show: tufte

#let bmat(..args) = math.mat(delim: "[", ..args)
#let vec(x) = math.bold(math.upright(x))

= 光照

光照和物质的相互作用会产生两种结果：_散射（scattering）_ 和 _吸收（absorption）_。当光线照射在介质分界处时，
散射会分开为_反射（reflection）_和_透射（transmission）_，同时有一部分光量被介质吸收。

反射光不集中在镜像方向，而是在较宽范围内出射，称为_漫反射（diffuse reflection）_。反射光是否密集，一般决定于
介质分界处的_表面粗糙度 (roughness)_。 光线经过某个介质分界处， 传播速度改变，导致观察出的传播方向发生偏折，
称为_折射（refraction）_。

#note[
三种不同的光线类型：
- 平行光源（太阳光）
- 点光源（_Point Light_）：光从一个点向四面八方发射。
- 聚光灯（_Spot Light_）：光只超一个方向的锥形范围发射。
][
  漫反射不一定发生在介质交界处（表面散射）。还有一部分漫反射光，来自透射光在介质体内的重新反射。
]


= 点着色

_冯氏光照模型（Phong Illumination Model）_记为：

$ I = I_"ambient" + I_"diffuse" + I_"specular" $

这是最常见的经验公式，不过并不是严格物理的。不便于描述材料区别，漫反射和镜面反射可随机叠加，
理论可能导致反射比入射更多的光。

== 镜面高光分量

$ I_"specular" = k_s I_l max(0, R dot V)^n $

其中：
- $V$ 表面点指向摄像机（观察者）的方向（direction toward viewerh)
- $R$ 是光线经过理想镜面反射方向
- $n$ 是光泽度指数（shiness exp) ,越大，高光越集中，越锐利。 
- $I_l$ 是光源强度
- $k_s$ 是材料的镜面反射系数。

```
        L        R
         \      /
          \    /
           \  /
            \/
            ▲ N
            |
────────────●──────── surface
             \
              \ V
               camera
```

在高亮分量下，R 和 V 的方向越近 $R dot V approx 1$，高光越大；反之就快速下降 $R dot V < 1$。

== 漫反射分量（Lambertian Shading）

$ I_"diffuse" = k_d I_l max(0, N dot L) $

- $N$ 是表面法线（surface normal）
- $L$ 是光源方向（direction toward light)，注意，$N dot L = cos theta$
- $k_d$ 是材料的漫反射系数

== 环境光分量

Phong Model 并不会计算光线的多次反射，即，若一个点背对光源，就会算出全黑。所以用该分量来补偿。

$ I_"ambient" = k_a I_a $ 

- $I_a$ 是假设的整个环境中都有的环境光强度。
- $k_a$ 是材质吸收、反射环境光的系数。

== 三角形着色

给定一个三角形几何，如何利用点着色方程，计算这个面上的着色。


- Gouruad Shading: 三个顶点计算点着色，然后内部像素仅做插值。
- Phong Shading: 三个顶点计算法线，计算内部像素的插值法线，然后内部像素独立计算点着色。

相对而言，Phong 计算量大，但是高光表现更好。



= 抗锯齿

抗锯齿（Anti-Aliasing, AA）是指减弱离散像素采样造成的边缘锯齿，使图形边界和细节过渡更平滑。
边缘锯齿通常是因为高分辨率信号以低分辨率显示时无法准确算出3D图形定位所造成的图形混叠（aliasing）
产生的。

== SSAA & MSAA 

Super-Sapmling AA 是指先对图形超分辨，然后再缩小为原采样率。开销最大，但效果最好。

== FXAA 

Fast Approximate AA 在渲染图像完成后，对图像进行边缘检测并模糊处理。很快。

== TAA 

Temporal AA 利用连续多帧的时间域信息进行采样。对运动中的细小几何锯齿效果很好，但是可能有拖影或模糊。

= 透明渲染

。。。

= BRDF

BRDF (Bidirectional Reflectance Distribution Function)



