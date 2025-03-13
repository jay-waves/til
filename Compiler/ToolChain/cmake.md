cmake 通过统一构建规则, 在不同平台上 (CentOS, Ubuntu, Android) 上预处理配置环境 (查找系统路径, 查找依赖及版本等), 然后生成具体的 Makefile.  

`CMakeLists.txt` 是 CMake 的配置文件. 

`cmake .` 来生成 makefile. 因为会同时在当前目录的各个子目录生成 CMakeFiles 和 CMakeCache.txt, 为了方便发布和管理, 通常在 src 同级目录下建一个 build 目录, 然后 `cmake ..` 此后执行 `make` 即可开始编译.

要重新构建时, 删除 build 目录即可.

### 基础配置

```cmake
cmake_minimum_required(VERSION 3.10) # 最低兼容 CMake 版本.
project(ProjectName) # 项目名称
```

### 设置变量

```cmake
set(MKL_POSSIBLE_PATHS
	"/opt/intel/mkl"
	"C:\\Program Files (x86)\\Intel\\onAPI\\mkl\\latest"
	"usr/local/mkl"
)

unset(MKL_POSSIBLE_PATHS)
```

### 添加外部依赖

链接外部依赖库:
```cmake
find_package(fmt CONFIG REQUIRED)
target_link_libraries(my_app my_library)

target_link_options(my_lib PRIVATE -Wl)
```

- `REQUIRED` 未找到时报错

### 添加源文件

使用 `add_executable` 和 `add_library` 添加源文件:

```cmake
add_executable(my_app main.cpp helper.cpp) # 定义可执行文件

add_library(my_lib STATIC src1.cpp src2.cpp) # 添加静态库

target_compile_options(my_lib PRIVATE -Wall -Wextra) # 添加编译选项
```

使用 `include_directories` 指定头文件目录
```cmake
include_directories(include)
```

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

### 测试

```cmake
enable_testing()

add_test(NAME test_app COMMAND test_app --option1 --option2 value) # 指定测试所用的脚本

set_tests_properties(test_app PROPERTIES TIMEOUT 30) # 设置属性, 如 超时 / 工作目录 / 环境变量 / 依赖
set_tests_properties(test_app PROPERTIES WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
set_tests_properties(test_app ENVIRONMENT "ENV_XXX=xxxx")
set_tests_properties(test_app PROPERTIES DEPENDS another_test_app)
```

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

## CMake 预定义变量

- `CMAKE_SOURCE_DIR` 顶级源码目录, 指向顶层 CMakeLists.txt
- `PROJECT_SOURCE_DIR` 子项目的源码目录
- `CMAKE_BINARY_DIR` 顶级构建目录, 即运行 CMake 配置命令的目录.
- `PROJECT_BINARY_DIR` 
- `CMAKE_SYSTEM_NAME`: Windows, Linux, Darwin 
- `CMAKE_SYSTEM_PROCESSOR`: x86_64, arm64
- `CMAKE_VERSION`
- `CMAKE_CROSSCOMPILING`
- `CMAKE_CXX_COMPILER_ID`: MSVC, GNU, Clang 
- `CMAKE_CXX_COMPILER`, `CAMKE_C_COMPILER`: 编译器路径.

> [CMake Basics](https://nu-msr.github.io/navigation_site/lectures/cmake_basics.html)