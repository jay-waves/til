ROS2 编译套件：
* rosdep 
* ament_cmake 
* colcon 

用 `rosdep` 检查和自动安装依赖：

```bash 
rosdep install -i --from-path src --rosdistro humble -y
```

用 `colcon` 构建整个仓库，一般而言，仓库的结构是：
* `./src/xxx` 存放所有子项目源码
* `./build` 
* `./install` 编译产物安装在这里，通过 `source ./install/setup.bash` 来导入环境。
* `log` 编译日志， ROS2 运行日志在 `~/.ros/log`

```bash 
colcon build --symlink-install \
		--packages-select <pkg> \
		# --packages-up-to <pkg>  \
		--parallel-workers 4 \
		--symlink-install 
```

推荐用 `colcon_defaults.yaml` 配置文件方式来精确控制编译参数

#### CMake 子项目

`./src/**` 下的每个文件夹是一个 CMake 子项目，同时有一个 `package.xml` 来指定 ROS2 相关的库依赖，供 `colcon, rosdep` 检查。子项目文件结构如下：
* `./src/*/CMakeLists.txt` 
* `./src/*/include/`
* `./src/*/package.xml`
* `./src/*/src/`

`CMakeLists.txt` 中，用 `ament_cmake` 来从 ROS2 环境中寻找依赖。

#### Python 子项目

子项目文件结构如下：基于传统 setuptools 
* `proj/package.xml` 
* `proj/resource/proj`
* `proj/setup.cfg` 
* `proj/setup.py` 
* `proj/proj/__init__.py`

uv 项目：
* `proj/pyproject.toml` 
* `proj/uv.lock`
* `proj/src/proj`

#### layers 

ROS 将依赖层次称为 layer 
* linux： layer0
* ros-humble：layer1
* navigation2: layer2
* robots based on nav2: layer3 

对 nav2 而言，ros-humble 是其 underlay，robots 是其 overlay 。在用 `source ./install/setup.bash` 时，需要按此顺序依次倒入环境。

