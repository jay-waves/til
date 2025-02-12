cmake 通过统一构建规则, 在不同平台上 (CentOS, Ubuntu, Android) 上预处理配置环境 (查找系统路径, 查找依赖及版本等), 然后生成具体的 Makefile.  类似的工具还有 configure.

`CMakeLists.txt` 是 CMake 的配置文件. 

`cmake .` 来生成 makefile. 因为会同时在当前目录的各个子目录生成 CMakeFiles 和 CMakeCache.txt, 为了方便发布和管理, 通常在 src 同级目录下建一个 build 目录, 然后 `cmake ..` 此后执行 `make` 即可开始编译.

要重新构建时, 删除 build 目录即可.

### 基础配置

```cmake
cmake_minimum_required(VERSION 3.10) # 最低兼容 CMake 版本.
project(ProjectName) # 项目名称
```

### 添加源文件

使用 `add_executable` 和 `add_library` 添加源文件:
```cmake
add_executable(my_app main.cpp helper.cpp)
```

使用 `include_directories` 指定头文件目录
```cmake
include_directories(include)
```

链接外部依赖库:
```cmake
find_package(... CONFIG REQUIRED)
target_link_libraries(my_app my_library)
```

- `REQUIRED` 指必须找到某个库, 否则会报错

### 添加动态链接库

假设 `a.cc` 为主文件, 依赖为: `a.cc -> b.cc -> c.cc`

```cmake
add_library(c c.cc)
add_library(b b.cc)

target_link_libraries(b private c)

add_executable(a a.cc)

target_link_libraries(a b c) # 必须放在目标文件产生之后
```

- `PRIVATE` 指 b 只被 c 需要, 不应该影响其他独立包.

### 构建

生成构建系统文件:
```bash
mkdir build
cd build
cmake .. # 注意当前在 build 路经中.
```

实际构建:
```bash
cmake --build .
```

系统环境变量 `CXX`, `CC` 指定了编译器, 一般不在 `CMakeList` 中指定.

添加编译参数:
```cmake
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -o3")
```

> [CMake Basics](https://nu-msr.github.io/navigation_site/lectures/cmake_basics.html)