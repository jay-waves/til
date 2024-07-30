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


## Bash 工具列表

[系统用户管理](sh/系统用户管理.md)

| `chmod` | `chown` |
| ------- | ------- |
| `getfacl`        |         |

[调试及技巧](sh/调试及技巧.md)

| `ldd`    |  | `dmesg` | `strace, ltrace` |
| -------- | ------ | ------- | ---------------- |
| `nm`     | `stap` | `stap`  | `perf`           |
| `sysdig` | `time` | `hyperfine`        |                  |

[进程管理](sh/进程管理.md)

| `pkill`  | `pgrep`   | `pstree` | `lsof`          | `(h)top`    |
| -------- | --------- | -------- | --------------- | --- |
| `fg, bg` | `jobs`    | `ps`     | `kill, killall` | `&` |
| `nohup`  | `timeout` |     |                 |     |
| `watch`  | `inotifywait`          |          |                 |     |

[配置及帮助](sh/配置及帮助.md)

| `man`     | `type`  | `whatis` |
| --------- | ------- | -------- |
| `whereis` | `which` |          |

[网络](sh/网络.md)

| `ip`                | `ping`      | `mtr`   | `netcat, socat` |
| ------------------- | ----------- | ------- | --------------- |
| `nslookup`          | `dig, host` | `whois` |                 |
| `netstat, ss`           | `slurm`        | `iftop` | `nethogs`                |
| `wireshark, tshark` |             |         |                 |
| `scp`               | `rsync`     | `ssh`   |                 |
| `curl`              | `wget`      |         |                 |
| `unshare`           | `firejail`  |         |                 |

[文本处理](sh/文本处理.md)

| `pandoc`           | `jq`                 | `shyaml`        | `xmlstarlet`   | `csvkit` |
| ------------------ | -------------------- | --------------- | -------------- | -------- |
| `hd, hexdump, xxd` | `hexedit, biew, bvi` | `(c)split`      | `strings`      |          |
| `awk`              | `sed`                | `grep, ripgrep` | `tokei`               |          |
| `cut, paste, join` | `echo, printf`       | `fmt`           |  `wc`             |          |
| `uniq`             | `sort`               | `nl`            | `iconv, uconv` | `expand` |
| `cat, bat`         | `diff`               | `tail, head`    | `more, less`   |          |

[文件系统](sh/文件系统.md)

|            | `mount`        | `fdisk`  | `mkfs`   | `lsblk` |
| ---------- | -------------- | -------- | -------- | ------- |
| `cd`       | `ls, tree`     | `mkdir`  | `pwd`    | `ln`    |
| `df`       | `du`           | `quota`  | `ldparm` |         |
| `find, fd`     | `locate`       |  `zoxide`        |          |         |
| `mv`       | `cp`           | `repren` | `rm`     | `touch` |
| `truncate` | `when changed` |          |          |         |
| `stat`     | `file`         | `chattr` | `wc`     |         |
| `xz`       | `7zip`         | `tar`    | `gzip`         |         |

[系统信息采集](sh/系统信息采集.md)

| `/proc`    | | `w`      | `uname` | `sysstat, dstat` |
| ---------- | -------- | -------- | ------- | ---------------- |
| `free`     | `lsxxx`  |          |         |                  |
| `screen`   | `tmux`   |          |         |                  |
| `shutdown` | `halt`   | `reboot` |         |                  |
| `uptime`   | `dateutils`         |          |         |                  |

[用户管理](sh/用户管理.md)

| `finger`   | `whoami`  | `last` | `passwd` | `id` |
| ---------- | --------- | ------ | -------- | ---- |
| `sudo, su` | `usermod` | `exit` | `groupmod`         |      |

[Bash 脚本编程](sh/编程/1%20变量.md)

| `export` | `env`   |     |     |     |
| -------- | ------- | --- | --- | --- |
| `tee`    | `xargs` | `history`    |     |     |

## 参考

- [awesome-shell](https://github.com/alebcay/awesome-shell) 一份工具列表.
- [awesome-osx-command-line](https://github.com/herrbischoff/awesome-osx-command-line): 一份针对 osX 的工具列表
- [Strict mode](http://redsymbol.net/articles/unofficial-bash-strict-mode/): bash 编程指导
- [shellcheck](https://github.com/koalaman/shellcheck): 静态 Shell 脚本分析工具, 本质是 bash/sh/zsh 的 lint
- [Filenames and Pathnames in Shell](http://www.dwheeler.com/essays/filenames-in-shell.html): 如何在脚本中正确处理文件名.
- [Data Science at the Command Line](http://datascienceatthecommandline.com/#tools)
