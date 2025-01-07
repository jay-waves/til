---
source: https://github.com/microsoft/vcpkg
---

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