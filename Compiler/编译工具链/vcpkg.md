---
source: https://github.com/microsoft/vcpkg
---

vcpkg 安装包时, 要对其平台 (triplets) 进行区分. 举例: `spdlog:x86-windows-static-md`
1. 指令集 (VCPKG_TARGET_ARCHITECTURE): x86, x64, amr, arm64, 
2. 平台 (VCPKG_CMAKE_SYSTEM_NAME): windows, linux, macOS, android, MinGW ...
3. 链接方式 (VCPKG_LIBRARY_LINKAGE): static, dynamic
4. 标准库类型 (VCPKG_CRT_LINKAGE): md, mt. 仅适用于 MSVC, mt 使用静态 CRT (`/MD, /MDd`), md 使用动态 CRT (`/MT, /MTd`).


## 与 VS 集成

编译并添加到环境变量中.
```powershell
git clone https://github.com/microsoft/vcpkg --depth 1
cd vcpkg 
bootstrap-vcpkg.bat
```

安装和 Miscrosoft Visual Studio 的集成, 这使得 VS 可以检测并使用被 vcpkg 安装的包.
```powershell
vcpkg integrate install
```

安装第三方库时, vcpkg 主要修改这些文件夹:
- packages
- downloads/tools
- buildtrees, 构建临时文件
- installed

## 与 VSCode 集成

配置 vcpkg: 
- `vcpkg new --applicaiton` 初始化项目, 会产生 vcpkg.json 和 vcpkg-configuration.json 两个文件
- `vcpkg add port xxx` 添加依赖

配置 cmake: 
1. 确保环境变量中有 `VCPKG_ROOT` 指向 vcpkg 安装位置
2. 修改 CMakePresets.json, 主要是设置变量 `CMAKE_TOOLCHAIN_FILE`. 详见下面的文件.

```json
{
    "version": 3,
    "configurePresets": [
        {
            "name": "default",
            "hidden": false,
            "generator": "Ninja",
            "binaryDir": "${sourceDir}/build",
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Debug",
                "CMAKE_TOOLCHAIN_FILE": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake",
                "CMAKE_C_COMPILER": "cl",
                "CMAKE_CXX_COMPILER": "cl"
            }
        }
    ]
}
```

配置 VSCode:
1. 确保电脑中有 MSVC 工具链
2. 用 Developer Powershell 打开 VSCode 
3. 安装 CMake, CMake Tools 插件. 然后构建当前目录.
	- 确保当前目录没有之前的缓存
	- `cmake --preset=default` 使用 CMkaePresets.json 中指定的构建工具
	- `ninja -C build` 
4. 观察 Output>cmake 的输出, 以及构建目录下 vspkg_isntalled 是否出现所需的第三方库.

```json
// settings.json
"terminal.integrated.defaultProfile.windows": "Developer PowerShell",
    "terminal.integrated.profiles.windows": {
        "Developer PowerShell" : {
            "source": "PowerShell",
            "icon": "terminal-powershell",
            "args": [
                "-NoExit",
                "-Command",
                "&{Import-Module 'C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\\Common7\\Tools\\Microsoft.VisualStudio.DevShell.dll'; Enter-VsDevShell b18f3dfd -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'}"
            ]
        }
    }
```