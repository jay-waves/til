
阶段一: 只有一个平台, 只需要一个编译器

- msvc
- gcc, 
- clang+llvm

阶段二: 不同操作系统链接过程不同, 配方不同

- make, 编译平台相关的 `Makefile`
- ninja, 编译平台相关的 `build.ninja`

阶段三: 元构建系统自动生成平台相关 makefile

- VisualStuido, 编译平台相关的 `.sln` (包含了阶段二和阶段三)
- cmake: 传统工具, 图灵完备. 代码为 `CMakeLists.txt`
- gn: Google 写的, 号称更快. 代码为 `BUILD.gn` , 代码风格更好.


现代项目构建流程:
```
元构建系统 --> 构建系统 --> 编译器
```