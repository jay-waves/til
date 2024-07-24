## Dockerfile

dockfile 定义了 docker engine 如何构建一个 docker 镜像, 构建过程为root用户. 可以参考我为 Rozz 设计的 [rozz dockerfile](../Robotic%20OS/fuzz.project/nav2/rozz%20dockerfile.md)

- `ENV`: 镜像构建和执行过程中都会存在参.
- `ARG`: 仅镜像构建中存在的参数.
- `RUN`: 执行命令. 默认使用 `/bin/sh -c`, ubuntu 指向 dash.

目录变化如下:
- docker镜像存储在本地 docker hub (/var/docker??)
- log...

dockerfile 构建技巧 (方便构建时 debug):
- 易改动指令放在后面, 声明参数等不易改动指令放在前面. 
- 尽量RUN指令合并多个命令, 比如将多个安装命令合并, 可以一次清理更多文件.
- 每个层次中, apt update 更新的源列表, 应该在使用完后立刻删掉, 来减小镜像体积. 因为本层不删除, 后续曾就无法删除(虽然只存储一次), 但是这样也会造成构建缓慢的问题.
- 注意, 宿主机构建命令的不同, 也会导致 dockerfile 重新构建

### 构建 dockerfile

`-t repos/image:tag`

```shell
docker build -t <image_name> .

docker run -it --name <coutainer_name> <image_name>

docker build -r /path/to/my/dockerfile .
```