* 260408 全面转向 UV 工具进行 Python 项目管理。

### 配置

通过：
* `UV_TOOL_BIN_DIR`
* `UV_TOOL_DIR`
* `UV_PYTHON_DIR` 
* `PATH` 

保存并删除原先 pip 的环境：

```bash
pip freeze > requirements.txt 
pip uninstall -y -r ./requirements.txt
```

### 项目配置

```bash 
uv init proj 
cd ./proj
```

推荐项目文件结构：
* `pyproject.toml` 
* `.python-version` 
* `.venv` 
* `uv.lock`
* `src/proj/__init__.py`

```bash 
uv add fastapi
uv add --dev ruff pytest
```

配置文件 `pyproject.toml` 

```toml
[project]
name = "xxx"
version = "0.1.0"
requires-python = ">=3.10"

dependencies = [
	"fastapi",
]

[project.optional-dependencies]
dev = [
    "pytest",
    "ruff",
]
```

创建完整环境：（创建 venv 以及安装依赖）

```bash 
uv sync

source .venv/bin/activate
```

导出包的接口 `__init__.py`

```python
from .client import Client 

__version__ = "0.1.0"

__all__ = [
	"Client",
]
```