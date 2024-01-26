# System Info

| cal    | date   |       |        |     |
| ------ | ------ | ----- | ------ | --- |
| df     | du     | quota |        |     |
| bg     | fg     | jobs  | ps     | (h)top |
| kill | killall| & | nohup | lsof |
| finger | whoami | last  | passwd |     |
| man    | uname       |       |        |     |

## 1 date

### `cal`

Shows the month's calendar.

### `date`

Shows the current date and time.

## 2 disk

### `df`

Shows disk usage.

### `du`

Shows the disk usage of files or directories. For more information on this command check this [link](http://www.linfo.org/du.html)

```bash
du [option] [filename|directory]
```

Options:

- `-h` (human readable) Displays output it in kilobytes (K), megabytes (M) and gigabytes (G).
- `-s` (supress or summarize) Outputs total disk space of a directory and supresses reports for subdirectories. 

Example:

```bash
du -sh pictures
1.4M pictures
```

### `quota`

Shows what your disk quota is.  

```bash
quota -v
```

## 3 process

### `bg`

Lists stopped or background jobs; resume a stopped job in the background.

### `fg`

Brings the most recent job in the foreground.

### `top`

Displays your currently active processes.

### `jobs`

Lists the jobs running in the background, giving the job number.

### `ps`

Lists your processes.  

```bash
ps -u yourusername

pstree -p
```

Use the flags ef. e for every process and f for full listing. 

```bash
ps -ef
```

###  `kill`

Kills (ends) the processes with the ID you gave.  

```bash
kill PID
```

### `killall`

Kill all processes with the name.  

```bash
killall processname
```

### `lsof`

list open files.

### &

The `&` symbol instructs the command to run as a background process in a subshell.

```bash
command &
```

### `nohup`

nohup stands for "No Hang Up". This allows to run command/process or shell script that can continue running in the background after you log out from a shell.

```bash
nohup command
```

Combine it with `&` to create background processes 

```bash
nohup command &
```

## 4 usr info

### `finger`

Displays information about user.  

```bash
finger username
```

### `whoami`

Return current logged in username.

### `last`

Lists your last logins of specified user.  

```bash
last yourUsername
```

### `passwd`

Allows the current logged user to change their password.

## other

### `uptime`

Shows current uptime. (已开机时间)

### `uname`

Shows kernel information.  

```bash
uname -a
```

### 检查系统状态

`df` dick free, 检查磁盘空间

`free` 检查主存空间

`vmstat $time_sec`, 如`vmstat 5` 列出一系列空间

### 关机

关机命令差别不大:

`shutdown`

`halt` 实际上调用了`shutdown -h`

`poweroff` 常用命令:
- `shutdown -h 30` 30分钟后自动关机
- `shutdown -h now` 立刻关机(root)
- `shutdown -r now` 立刻重启(root)
- `shutdown -r 00:30` 在00:30时候重启(root)
- `init 0` 也可以关机, `init 6`也是重启

重启: `reboot`

#### 关机流程

1. 使用`who`查看主机是否有还在线用户
2. 使用`netsat -a`确定是否有网络连接
3. 使用`ps -aux`查看后台进程状态
1. 使用数据同步`sync`
2. 关机### 检查系统状态
