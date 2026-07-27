> It has always been the spirit of Unix to have separate programs 
> that do their job well, and work together to perform a bigger task.
> 
> "Do one thing and do ti well"  -- philosophy of Unix

## 获取帮助

* `man <command>`
* `tldr <command>` 


## Bash IO

详见 [bash io redirect](./bash-io.md)

## Bash Variables

Bash 变量包含继承自父程序的**环境变量**和局部变量。变量相关内容详见 [bash variables](./bash-var.md)

bash 有两种展开类型：
* `${var}` 变量展开
* `$(instru)` 子命令替换


另一种子命令是 sub-shell，其对工作目录或变量的改动不会影响当前 shell:

```bash
(cd /tmp && command)
```

## Bash Script

Bash 有脚本能力，但是并不推荐写，隐蔽行为太多。越自作聪明，调试代码就越痛苦。   

### Shebang

在 Linux 脚本文件中，第一行用于指定脚本的执行程序，称为 `shebang`:

```bash
#!/usr/bin/env bash -x 

# bash -x: print the dry-run commands
```

```bash
#!/usr/bin/python
```

### Funciton

```bash
foo() {
    local arg=$1
    echo "$arg"
    return 0
}
```

* 参数：`$1`、`$2`、`"$@"`
* 函数内声明变量优先用 `local`

function 默认不接收 `stdin`，需要手动用 `read` 接收调用者可能传入的 `stdin`

```bash
read_input() {
    local input
    read -r input
    echo "$input"
}

echo "hello" | read_input
```

### Condition


```bash
[[ ... ]]   # 字符串、文件条件
(( ... ))   # 数值条件
```

```bash
if [[ $name == admin* ]]; then
    ...
fi

if (( count > 10 )); then
    ...
fi
```

常用判断：

```bash
[[ -n $var ]]       # 非空
[[ -z $var ]]       # 为空
[[ -e $file ]]      # 存在
[[ -f $file ]]      # 普通文件
[[ -d $file ]]      # 目录
[[ -x $file ]]      # 可执行
```

命令退出码可直接作为条件：

```bash
command && success
command || failure
```

### Loop

```bash
for item in "${items[@]}"; do
    echo "$item"
done
```

```bash
while IFS= read -r line; do
    echo "$line"
done < file.txt
```

```bash
for (( i = 0; i < 10; i++ )); do
    ...
done
```


## Bash Jobs

### background jobs

```bash
command &
pid=$!

jobs -l
wait "$pid"
wait
```

```bash
Ctrl-Z      # 暂停前台任务
bg %1       # 后台继续
fg %1       # 切回前台
kill %1     # 结束 job
```

### find a process

```bash
ps -ef
pstree -p

pgrep -af pattern
pkill -TERM -f pattern

kill -TERM "$pid"
kill -KILL "$pid"   
```

优先 `SIGTERM`，少用 `kill -9`。

### cleanup signals

```bash
pids=()

cleanup() {
    kill "${pids[@]}" 2>/dev/null
    wait "${pids[@]}" 2>/dev/null
}

trap cleanup EXIT INT TERM

worker &
pids+=("$!")
```

### parallel jobs

```bash
command1 &
command2 &
wait
```

```bash
printf '%s\0' *.jpg | xargs -0 -n1 -P8 convert
```

复杂并行任务使用 GNU Parallel：

```bash
parallel -j8 'convert {} {.}.png' ::: *.jpg
```

## Bash 热键

- `Ctrl-r`: 查找历史命令
- `Tab`: 补全
- `Ctrl-a, Ctrl-e`: 行首, 行尾

更推荐用 vim 模式来编辑长命令。