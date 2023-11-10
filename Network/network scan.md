1. 路由追踪
2. 网络扫描
3. 端口扫描

### 端口扫描

查看端口是否被占用：

`netstat -tuln | grep [port]`, (需安装 net-tools)

`lof -i :[port]` list open files, 查看端口占用
