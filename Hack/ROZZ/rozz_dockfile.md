镜像: navigation2 镜像衍生开 asan 和 test 的定制 nav2 镜像.

```dockerfile
# ros humble, based on jammy.
# see: https://github.com/docker-library/repo-info/blob/master/repos/ros/remote/humble.md
FROM ros@sha256:6bdb91d8f1c7177afb834737d281caf3515965ebc24053eab04044f97568eadb

# no interactive install
ENV DEBIAN_FRONTEND=noninteractive

# install build and debug tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    lcov \
    clang \
    libasan5 \
    && pip install \
    gcovr \
    && rm -rf /var/lib/apt/lists/*

# add ros2 humble apt lists
RUN apt-get update && apt-get install -y software-properties-common \
	&& add-apt-repository universe \
	&& apt-get update && apt-get install -y curl 
	
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
	&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# setup proxy
ARG PROXY="http://192.168.1.107:7890"
# ENV http_proxy=$PROXY
# ENV https_proxy=$PROXY
RUN git config --global http.proxy $PROXY
RUN git config --global https.proxy $PROXY

############## 编译 nav2, nav2 as overlay, ros2 as underlay
# setup workspace
RUN mkdir -p /root/nav2_ws/src
ENV OVERLAY_WS=/root/nav2_ws
WORKDIR $OVERLAY_WS
# clone source code
RUN git clone https://github.com/ros-planning/navigation2.git --branch humble ./src/navigation2

# source underlay of ros2 humble
ARG SOURCE_UNDERLAY='source /opt/ros/humble/setup.bash'
# ARG SOURCE_OVERLAY='source $OVERLAY_WS/install/setup.bash'
# ENV COVERAGE_COMMAND="lcov --directory . --capture --output-file coverage.info"

# install navigation2 dependencies
RUN /bin/bash -c "$SOURCE_UNDERLAY \
    && apt update \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src"

# compiler flags
ENV CC=/usr/bin/clang
ENV CXX=/usr/bin/clang++
ENV CMAKE_CXX_STANDARD=17
ENV CMAKE_CXX_FLAGS="-w -Wno-error -Wno-format-security"
ENV CMAKE_C_FLAGS="-w -Wno-error -Wno-format-security"
# ENV BUILD_TESTING=OFF
# ENV MAKEFLAGS='-j4'

# build
# if resources are limited, use --parallel-workers 2
RUN /bin/bash -c "$SOURCE_UNDERLAY && colcon build --symlink-install"
```

`docker build -t basic_nav2 . --network=host`

```dockerfile
# 基于上一个 dockerfile 构建的nav2
from ...
############### 开 ASan 和 Coverage 编译单个包
ENV LDFLAGS="-fsanitize=address"
ENV CMAKE_C_FLAGS="$CMAKE_CXX_FLAGS -fsanitize=address -fno-omit-frame-pointer --coverage -DCOVERAGE_RUN=1"
ENV CMAKE_CXX_FLAGS="$CMAKE_CXX_FLAGS -fsanitize=address -fno-omit-frame-pointer --coverage -DCOVERAGE_RUN=1"
ARG ASAN_PACKS="nav2_amcl nav2_bt_navigator"
RUN /bin/bash -c '$SOURCE_UNDERLAY && colcon build --symlink-install --packages-select $ASAN_PACKS --cmake-clean-cache'

```

运行时镜像: 删除编译工具等.
```dockerfile
from ...
############### 配置运行时环境 (用bashrc)
ENV ASAN_OPTIONS="new_delete_type_mismatch=1 detect_leaks=1 halt_on_error=0"
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ENV TURTLEBOT3_MODEL=waffle

ENTRYPOINT ["/bin/bash", "-c", "$SOURCE_UNDERLAY && $SOURCE_OVERLAY && ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True composition:=False"]
```


### 配置 ros2 进程间通信

`ROS_HOSTNAME`

`ROS_MASTER_URL`: `http://<docker-container-ip>:11311`

`ROS_IP