## 虚拟环境

创建一个解释器隔离环境, 独立管理项目依赖.

建立虚拟环境: `python -m venv /path/to/venv`

可以使用系统包的轻量虚拟环境: `python -m venv <path> --system-site-packages`

虚拟环境有如下内容:
- 新 `site-packages` 目录, 安装 Python 包, 同时不影响全局安装.
- `pip` 副本.
- 一些脚本, 如 `activate`, 用于激活 venv.

### 激活

当运行 venv 中的 Python 解释器时, `sys.prefix` 和 `sys.exec_prefix` 将指向该 venv 的目录, 而 `sys.base_prefix` 和 `sys.base_exec_prefix` 将指向用于创建该 venv 的基础 Python 的目录.  

**检测 `sys.prefix != sys.base_prefix` 可确定当前解释器是否运行于虚拟环境中.**

### 保存环境

保存当前 venv 中所有包和版本: `pip freeze > requirements.txt`

相应的安装方式: `pip install -r requirements.txt`

## 离线

当 pip 没有适合的包版本时, 可使用 wheel.exe 来下载 `.whl` 安装包.
1. 下载对应 `.whl` 文件
2. `pip install /pth/to/install`

查询当前 python 解释器的环境 (py 版本 + 平台)
```python
>>> import platform
>>> platform.architecture()
'''("64Bit","WindowsPE")'''
```