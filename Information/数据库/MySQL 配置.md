windows 上配置: 无安装版, 即直接下载 zip 打包程序.

## 1 配置服务

> mysql手册 2.3.4, 以下均需要管理员权限操作

将二进制程序包直接解压在 `C:\mysql` 中, *无二级目录*. 并创建 `my.cnf`.

初始化:

1. 添加 `C:\mysql\bin` 目录入 `PATH`
2. 初始化数据文件夹 `mysqld --initialize`, 安装目录多出了 `mysql\data`
3. 启动mysqld `mysqld --console`, `--console` 使 mysqld 保持前台运行, 日志输出到命令行, 而不是`mysql\data`
4. 关闭: `mysqladmin -u root shutdown -p`, `-p` 开启密码验证.

安装windows服务:

添加 mysql 服务: `mysqld.exe --install-manual`, `-manual` 防止服务开启自启.

开启service: `net start MySQL`, **此方法不推荐. windows有棘手的用户权限问题.**

### 另一种方法

在解压目录创建 `mysql.ini`: 注意替换 datadir 和 basedir 为实际解压路径.

```toml
[mysqld]
datadir=F:/myserver/mysql/data
basedir=F:/myserver/mysql
port=3306
bind-address=0.0.0.0
skip-grant-tables

[client]
port=3306
```

然后以管理员身份启动 cmd, 导航至解压目录的 `/bin` 目录, 初始化 mysql. 结束后可观察到 data 文件夹被创建.

```cmd
mysqld --initialize-insecure --user=mysql
```

将 mysql 安装为 windows 服务: 
```cmd
mysqld --install-manual MySQL --defaults-file=F:\myserver\mysql\mysql.ini
```

### 若 mysql 服务已存在, 需重装

先删除服务, 再重装.

```cmd
> sc delete MySQL
[SC] DeleteService SUCCESS
```

或 powershell

```powershell
Remove-Service -Name "MySQL"
```

## 2 客户端登录

`mysql -h localhost -P 3306 -u root -p`
- `-u` 用户名
- `-p` 开启密码验证
- `-h` 客户端主机地址, 此时开启TCP/IP连接.
- `-P` 端口

## 3 用户管理

重置默认 root 密码: 
```sql
ALTER USER 'root'@'localhost' IDENTIFIED BY '123456';
FLUSH PRIVILEGES;
```

**mysql会将某些表缓存在内存中, 必须用`FLUSH`强制刷新, 对数据的改动才会生效.**

新用户: 这个新用户绑定了一个来源主机.
```sql
-- 允许本地user:
CREATE USER 'newuser'@'localhost' IDENTIFIED BY 'newpassword';
-- 允许来自所有主机的user:
CREATE USER 'newuser'@'%' IDENTIFIED BY 'newpassword';
```

注意 mysql 中 localhost 行为/权限和 127.0.0.1 有区别. 用 127.0.0.1 连接 mysql 时, 会使用 TCP/IP 方式连接数据库;  当用 localhost 连接 mysql 时, 通过 Unix socket 方式连接数据库.

所以当使用 localhost 时, `my.cnf` 中不要开启 `skip-name-resolve`. 否则 mysql 无法通过主机名来识别客户端, 只能将主机名解析为地址形式 127.0.0.1, 但是 127.0.0.1 和 localhost (用户, 权限) 的配置是完全分开的, 导致连接错误. #Debug 

> 详见 [mysql本地连接-blog](https://blog.51cto.com/wushank/1685041)


|      | localhost                  | 127.0.0.1                    | 本机IP                       |
| ---- | -------------------------- | ---------------------------- | ---------------------------- |
| 网络 | 不联网                     | 不联网                       | 联网                         |
| 传输 | 不使用网卡, 不受防火墙限制 | 网卡传输, 受防火墙和网卡限制 | 网卡传输, 受防火墙和网卡限制 |
| 访问 | 本机访问                   | 本机访问                     | 本机或外部访问                             |

授权:
```sql
-- 所有权限:
GRANT ALL PRIVILEGES ON *.* TO 'newuser'@'localhost';
-- 只给特定数据库
GRANT ALL PRIVILEGES ON database_name.* TO 'newuser'@'localhost';
```

配置新数据库:
```sql
CREATE DATABASE my_new_database;
-- 选择该库
USE my_new_database;
```

执行sql脚本:
```sql
source /path/to/cmd.sql
```