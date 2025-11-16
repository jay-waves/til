> It has always been the spirit of Unix to have separate programs 
> that do their job well, and work together to perform a bigger task.
> 
> "Do one thing and do ti well"  -- philosophy of Unix

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

### `which`

模拟搜索 PATH 查询可执行文件的过程, 若成功, 显示完整路径. (不能检查其他类型命令!) 可用于检查当多个重名程序存在于 PATH 中时, bash 究竟执行了哪一个.

```bash
$ which php
/c/xampp/php/php
```

### BashScript

Bash 作为编程语言的语法和细节请见: [BashScript](bash%20script/ReadMe.md).

## Bash 工具列表

Linux 下有四种命令类型 (用 `type` 查看)
- `/bin` 文件夹下的可执行程序, 一般用 PATH 索引
- shell builtins 
- shell functions (scripts)
- alias

[进程调试](../可观测性/进程调试.md)

| `ldd`    |  | `dmesg` | `strace, ltrace` |
| -------- | ------ | ------- | ---------------- |
| `nm`     | `stap` | `stap`  | `perf`           |
| `sysdig` | `time` | `hyperfine`        |                  |

[多任务控制](多任务控制.md)

| `pkill`  | `pgrep`   | `pstree` | `lsof`          | `(h)top`    |
| -------- | --------- | -------- | --------------- | --- |
| `fg, bg` | `jobs`    | `ps`     | `kill, killall` | `&` |
| `nohup`  | `timeout` |     |                 |     |
| `watch`  | `inotifywait`          |          |                 |     |

配置及帮助

| `man`     | `type`  | `whatis` |
| --------- | ------- | -------- |
| `whereis` | `which` |          |

[网络调试](../可观测性/网络调试.md)

| `ip`                | `ping`      | `mtr`   | `netcat, socat` |
| ------------------- | ----------- | ------- | --------------- |
| `nslookup`          | `dig, host` | `whois` |                 |
| `netstat, ss`           | `slurm`        | `iftop` | `nethogs`                |
| `wireshark, tshark` |             |         |                 |
| `scp`               | `rsync`     | `ssh`   |                 |
| `curl`              | `wget`      |         |                 |
| `unshare`           | `firejail`  |         |                 |

[文本文件处理](文本文件处理.md)

| `pandoc`           | `jq`                 | `shyaml`        | `xmlstarlet`   | `csvkit` |
| ------------------ | -------------------- | --------------- | -------------- | -------- |
| `hd, hexdump, xxd` | `hexedit, biew, bvi` | `(c)split`      | `strings`      |          |
| `awk`              | `sed`                | `grep, ripgrep` | `tokei`               |          |
| `cut, paste, join` | `echo, printf`       | `fmt`           |  `wc`             |          |
| `uniq`             | `sort`               | `nl`            | `iconv, uconv` | `expand` |
| `cat, bat`         | `diff`               | `tail, head`    | `more, less`   |          |

[文件系统](文件系统.md)

|            | `mount`        | `fdisk`  | `mkfs`   | `lsblk` |
| ---------- | -------------- | -------- | -------- | ------- |
| `cd`       | `ls, tree`     | `mkdir`  | `pwd`    | `ln`    |
| `df`       | `du`           | `quota`  | `ldparm` |         |
| `find, fd`     | `locate`       |  `zoxide`        |          |         |
| `mv`       | `cp`           | `repren` | `rm`     | `touch` |
| `truncate` | `when changed` |          |          |         |
| `stat`     | `file`         | `chattr` | `wc`     |         |
| `xz`       | `7zip`         | `tar`    | `gzip`         |         |

[系统信息](../sss/系统信息.md)

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

[Bash 脚本编程](bash%20script/变量.md)

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
