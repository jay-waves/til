> It has always been the spirit of Unix to have separate programs 
> that do their job well, and work together to perform a bigger task.

## 获取帮助

### `man`

```sh
man <command>

man -k           # 基于关键词检索, 等价于 apropos
man ascii
man unicode 
man utf-8

info
help
Get-Help          # Powershell 专属

tldr   # too long don't read, 基于例子的命令手册. 需要在线使用.

cht.sh  # 类似 tldr, 不过语义更智能, 还能查各语言文档. 
curl cht.sh/python/what+is+venv 

<command> -h
<command> -Help    # Powershell 专属
<command> --help
<command> /h       # CMD 命令参数
```

### `type`

`type command` 显示某个命令的类型, linux中命令有四种类型:
- /bin文件夹中的可执行程序
- shell builtins内置程序
- shell function即scripts
- alias 别名

<pre>
$ type cd
cd is a shell builtin
</pre>

### `whatis`

显示单行, 非常简短的命令介绍

<pre>
$ whatis bash
bash (1)             - GNU Bourne-Again SHell
</pre>

### `whereis`

whereis searches for executables, source files, and manual pages using a database built by system automatically.

Example:

```bash
$ whereis php
/usr/bin/php
```

### `which`

模拟搜索 PATH 查询可执行文件的过程, 若成功, 显示完整路径. (不能检查其他类型命令!) 可用于检查当多个重名程序存在于 PATH 中时, bash 究竟执行了哪一个.

```bash
$ which php
/c/xampp/php/php
```

### BashScript

Bash 作为编程语言的语法和细节请见: [BashScript](BashScript/ReadMe.md)

## Bash 工具列表

[文件权限](文件权限.md)

| `chmod` | `chown` |
| ------- | ------- |
| `getfacl`        |         |

[进程及程序调试](进程及程序调试.md)

| `ldd`    |  | `dmesg` | `strace, ltrace` |
| -------- | ------ | ------- | ---------------- |
| `nm`     | `stap` | `stap`  | `perf`           |
| `sysdig` | `time` | `hyperfine`        |                  |

[进程任务管理](进程任务管理.md)

| `pkill`  | `pgrep`   | `pstree` | `lsof`          | `(h)top`    |
| -------- | --------- | -------- | --------------- | --- |
| `fg, bg` | `jobs`    | `ps`     | `kill, killall` | `&` |
| `nohup`  | `timeout` |     |                 |     |
| `watch`  | `inotifywait`          |          |                 |     |

配置及帮助

| `man`     | `type`  | `whatis` |
| --------- | ------- | -------- |
| `whereis` | `which` |          |

[网络调试](网络调试.md)

| `ip`                | `ping`      | `mtr`   | `netcat, socat` |
| ------------------- | ----------- | ------- | --------------- |
| `nslookup`          | `dig, host` | `whois` |                 |
| `netstat, ss`           | `slurm`        | `iftop` | `nethogs`                |
| `wireshark, tshark` |             |         |                 |
| `scp`               | `rsync`     | `ssh`   |                 |
| `curl`              | `wget`      |         |                 |
| `unshare`           | `firejail`  |         |                 |

[文件内容](文件内容.md)

| `pandoc`           | `jq`                 | `shyaml`        | `xmlstarlet`   | `csvkit` |
| ------------------ | -------------------- | --------------- | -------------- | -------- |
| `hd, hexdump, xxd` | `hexedit, biew, bvi` | `(c)split`      | `strings`      |          |
| `awk`              | `sed`                | `grep, ripgrep` | `tokei`               |          |
| `cut, paste, join` | `echo, printf`       | `fmt`           |  `wc`             |          |
| `uniq`             | `sort`               | `nl`            | `iconv, uconv` | `expand` |
| `cat, bat`         | `diff`               | `tail, head`    | `more, less`   |          |

[文件系统管理](文件系统管理.md)

|            | `mount`        | `fdisk`  | `mkfs`   | `lsblk` |
| ---------- | -------------- | -------- | -------- | ------- |
| `cd`       | `ls, tree`     | `mkdir`  | `pwd`    | `ln`    |
| `df`       | `du`           | `quota`  | `ldparm` |         |
| `find, fd`     | `locate`       |  `zoxide`        |          |         |
| `mv`       | `cp`           | `repren` | `rm`     | `touch` |
| `truncate` | `when changed` |          |          |         |
| `stat`     | `file`         | `chattr` | `wc`     |         |
| `xz`       | `7zip`         | `tar`    | `gzip`         |         |

[系统硬件信息](系统硬件信息.md)

| `/proc`    | | `w`      | `uname` | `sysstat, dstat` |
| ---------- | -------- | -------- | ------- | ---------------- |
| `free`     | `lsxxx`  |          |         |                  |
| `screen`   | `tmux`   |          |         |                  |
| `shutdown` | `halt`   | `reboot` |         |                  |
| `uptime`   | `dateutils`         |          |         |                  |

用户管理

| `finger`   | `whoami`  | `last` | `passwd` | `id` |
| ---------- | --------- | ------ | -------- | ---- |
| `sudo, su` | `usermod` | `exit` | `groupmod`         |      |

[Bash 脚本编程](BashScript/变量.md)

| `export` | `env`   |     |     |     |
| -------- | ------- | --- | --- | --- |
| `tee`    | `xargs` | `history`    |     |     |

## Bash 热键

查看 `man readline`

- `Ctrl-r`: 查找历史命令
- `Tab`: 补全
- `Ctrl-a, Ctrl-e`: 行首, 行尾
- `Alt-f, Alt-b`: 前进一单词, 后退一单词
- `Alt-*` 展开通配符匹配项
- `Ctrl-f, Ctrl-b`: 前进一字符, 后退一字符
- `Ctrl-u`: 删除光标前内容
- `Ctrl-w`: 删除光标前单词
- `Ctrl-d`: 删除当前字符
- `Ctrl-l`: 清屏
- `Alt [0-9] Alt .` 黏贴上一条命令的最后`[1-9]`个参数
- `Ctrl-x`, `Ctrl-e` 使用系统默认编辑器编辑命令
- `Ctrl-z` 将命令移至后台, 后可用 `fg` 调回
