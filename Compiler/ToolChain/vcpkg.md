---
source: https://github.com/microsoft/vcpkg
---

## 初始化项目

环境: Debian, X86-64

初始化:
```bash
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg && ./bootstrap-vcpkg.sh
export PATH=/path/to/vcpkg:$PATH
```

配置 CMake 兼容 vcpkg:

- `CMakePresets.json` (CMake>3.19) 
```json
{
  "version": 3,
  "configurePresets": [
    {
      "name": "vcpkg",
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build",
      "cacheVariables": {
        "CMAKE_TOOLCHAIN_FILE": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake",
        "CMAKE_EXPORT_COMPILE_COMMANDS": "ON",
      }
    }
  ]
}
```

- `CMakeUserPresets.json`: 该文件应该加入 `.gitignore`, 仅用于本地配置. 
```json
{
    "version": 3,
    "configurePresets": [
      {
        "name": "default",
        "inherits": "vcpkg",
        "environment": {
          "VCPKG_ROOT": "path/to/vcpkg"
        }
      }
    ]
  }
```

构建:

```bash
cmake -S . -B build -G Ninja --preset default 
cmake --build build
...
```