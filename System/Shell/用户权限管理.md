### `chmod`

改变文件(夹)的读/写/执行权限. Linux 中权限以权限位形式出现, 如文件权限为可能是 `rwx rwx r--`, 第一个 `rwx` 为属主权限, 第二个 `rwx` 为群组权限, 第三个 `rwx` 为访客权限, 无权限用 `-` 占位. 

```bash
chmod -options filename
```

`rwx` 字母分别对应数字权限 `124`, 权限简写为三者之和. 如 `rw-` 对应 `3`.

设置权限为 `rwxr-xr-x`, 如下:

```bash
# 设原本权限为 r-xr-xr-x, 给用户添加写权限:
# u 代表用户, g 代表群组
chmod u+w file 

# 或直接:
chmod 755 file
```

将某个文件夹递归设置权限: 

```bash
chmod -R 777 path
```

设置新建文件权限: `umask 000`, umask 是一个权限掩码, 请求的权限码和 umask 进行按位与非, 生成最终权限码 (即 umask 中定义的权限被移除. 仅在临时会话有效. 系统默认 `umask 022`, 在 /etc/profile 中.

#### `s` 位

`chmod u+s ...` 指 setuid, 无论谁执行该文件, 程序都会自动以文件所有者的权限运行. 用于需要提供临时提权的程序. 

`chmod g+s ...` 指 setgid, 无论谁执行该文件, 都会以文件属组的权限运行. 设置在目录时, 目录下所有新文件都会继承此目录的组, 而非创建者的组.

此时程序权限如: `rws...`

#### `t` 位

也称粘滞位, 设置在目录上时, 即使多个用户都有对目录的写权限, 但只有文件的所有者或超级用户才能**删除或重命名**该目录.

此时权限如: `drwxrwx--T`

### `chown` !

change owner. 改变文件属主.

```bash
chown -options user:group filename
```

### `getfacl`

保存和恢复文件权限.

```bash
   getfacl -R /some/path > permissions.txt
   setfacl --restore=permissions.txt
```

### `finger`

`finger username` 展示用户信息

### `whoami`

展示当前登入用户的用户名.

### `last`

`list <username>` 展示该用户上次登入信息.

### `passwd`

修改密码

### `id`

用户/组 ID 信息

## 用户管理

修改主机名 `sudo hostnamectl set-hostname <host_name>`

切换用户 `su [- username]`, 加入 `-` 会使切换后的环境与使用该用户登陆后的环境相同, 省略 `username` 默认登录 root.

### 更改用户名

1. `su`
2. `vim /etc/passwd`
3. `vim /etc/shadow`
4. `vim /etc/group`

删除用户 `sudo userdel -r <usr_name>`

修改用户名: (最好使用临时sudo用户进行操作)
- `sudo usermod -l <new_usr> -m <old_usr>`
- `sudo groupmod -n <new_group> <old_group>`
- `sudo mv /home/<old> /home/<new>`, 并修改 `/etc/passwd` (如果未自动修改)

### 创建新用户

创建新用户: `sudo useradd -m -s /bin/bash <usr_name>`
- `-m` 创建用户家目录
- `-s /bin/bash` 默认 shell

设置新用户密码: `sudo passwd <usr_name>`

为新用户添加sudo权限: `sudo usermod -G sudo <usr>`

### 使用 root 用户

使用`su`命令进入管理员模式(`#`)

如果无法使用su (即无法使用 root), 那么需要初始化root密码: `sudo passwd`

使用`exit`退出管理员模式

### 将用户添加到 wheel 组

sudo 是一个提权的程序, 不同系统用不同用户组来标识用户的调用 sudo 权限.

在 Debian 系统版本中, 通过 sudo 用户组来控制用户 sudo 权限, 这些系统预设了 `etc/sudoers` 的用户组权限设置.

```
%sudo ALL=(ALL:ALL) ALL
```

基于 Arch 和 Fedora 的发行版, 则更为传统地使用 wheel 用户组而不是 sudo. 但这些系统一般不默认开启 `etc/sudoers` 中的设置.

```
$ visduo
# %wheel ALL=(ALL) ALL
```

