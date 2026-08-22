
#import "../appx/theme.typ": tufte, note, mermaid
#show: tufte

#let bmat(..args) = math.mat(delim: "[", ..args)
#let vec(x) = math.bold(math.upright(x))

图形渲染管线：

```mermaid
flowchart LR;
  Application-->Geometry-->Rasterizer;
``` 

= Geometry Statge 

几何渲染阶段：

```mermaid
flowchart LR;
  A[View Transform]-->B[Vertex Shading]-->C[Projection]-->D[Clipping]-->E[Screen Mapping];
``` 

== 模型和视图变换

以相机为原点，重建所有物体的坐标。便于后续几何渲染。

== 顶点着色

_着色 (shading)_ 是指确定虚拟材质上的光照、颜色、纹理效果。几何阶段，只进行模型的顶点着色，
其他着色则在每个像素_光栅化（Rasterization）_期间执行。

== 投影

投影操作要将模型变换到一个单位立方体（Unit Cube）内，然后进行：
- 正交投影 (Orthographic Projectino) 投影变换后，视线仍保持水平。
- 透视投影 (Perspective Projection) 投影变换后，视线汇聚一点 

投影后，模型的显示状态变成二维。

== 裁剪

将投影后，不出现在视野内的图元删除或裁切，不发送到光栅化阶段。

== 屏幕映射 

将虚拟世界坐标系，映射为屏幕坐标系，然后将像素放入光栅化阶段。

= Rasterizer Stage 

光栅化阶段：
+ 三角形配置与计算（Triangle）：找到每个像素所属的三角形。
+ 像素着色（Pixel Shading）：根据三角形顶点的着色数据（几何阶段）来计算像素着色。
+ 融合（Merging）：将每个像素的各类颜色缓冲区数据融合（如 RGBA、Z缓冲）。

```mermaid
flowchart LR;
  A[Triangle Setup]-->B[Triangle Traversal]-->C[Pixel Shading]-->D[Merging];
```

