
#import "@local/ypst-template:0.1.0": template, sidenote, mermaid
#show: template

#let bmat(..args) = math.mat(delim: "[", ..args)
#let vec(x) = math.bold(math.upright(x))

图形渲染管线主要分为两部分：Geomotry --> Rasterizer，几何阶段 --> 光栅化阶段。 

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

= GPU Pipeline 

```mermaid
flowchart LR;
  A[Vertex Shader]-->B[Clipping]-->C[Screen Mapping]-->D[Triangle]-->E[Pixel Shader]-->F[Merger]
```

其中 Vertex Shader (VS) 和 Pixel Shader (PS) 是可编程的，其他的受限。着色阶段有独立的编程语言，如 GLSL、HLSL、Slang，
这些着色语言最终会被翻译为 GPU 硬件支持的汇编语言。

```
                    GPU Program
                        │
                  Execution Stage
                        │
                     Dispatch
                        │
                    Workgroup
                        │
                     Subgroup
                        │
                      Lanes
                        │
                   Invocations
                        │
          ┌─────────────┼─────────────┐
          │             │             │
       private       shared        global
        state         state         state
```

在逻辑上，一个 Shader 程序被拆分为多个 `invocation` 任务并行执行，在底层（硬件层），
`invocation` 被映射到 GPU SIMD 执行单元 `lane`，多个 `lane` 被划归一个 `warp` 执行组。

这个概念在不同 GPU/API 上的称呼不同：

#table(
  columns: 3,
  table.header([Brand], [Invocation Group], [Invocation Insight],),
  [NVIDIA, CUDA], [Warp], [Lane], 
  [AMD], [Wave], [Lane], 
  [Intel], [Subgroup], [Lane],
  [Vulkan, GLSL], [Subgroup], [Invocation],
  [DirectX], [Wave], [Lane],
)

一个执行组内的所有 `invocations` 是并行执行的，遇到*条件语句*时，需要用掩码屏蔽一部分
不符合条件的 `invocations`。



Shader 程序的资源模型：

```
                     Host / CPU
                          │
                   Resource Binding
                          │
        ┌─────────────────┼───────────────┐
        │                 │               │
     Uniform Buffer     Texture       Storage Buffer
     Constant Buffer    Sampler       Storage Image
        │                │               │
        └────────────────┼───────────────┘
                         ▼
                  Shader Invocation
                        │
        ┌───────────────┼────────────────┐
        │               │                │
     private         workgroup         device
     memory           memory           memory
```

Shader 程序有两种不同的变量：
- Uniform ：不变的，被所有线程共享的数据 
- Varying ：私有的、可变的数据，每个线程独有

= Deferred Rendering 


