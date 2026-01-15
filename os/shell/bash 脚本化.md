不推荐使用 BashScript 写复杂程序. 

## 实例

对文本进行组合操作:

```bash
	sort a b | uniq > c   # c 是 a 并 b
	sort a b | uniq -d > c   # c 是 a 交 b
	sort a b b | uniq -u > c   # c 是 a - b
```

检查目录下所有文件的内容:

```bash
	grep . *     # 即匹配目录下的任意文件内容, 按行输出
	head -100 *  # 按文件输出
```

sh 处理文本行列极便捷. 如计算文本文件第三列之和:

```bash
	awk '{ x += $3 } END { print x }' myfile
```

查看文件树+详细文件信息: (类似递归版的 `ls -l`)

```bash
	ls -lR
# or:
	find . -type f -ls
```

access.log 是 web 服务器的日志文件, 某个确定的值仅出现在某些行中 (如 acct_id 参数总在 URI 中). 计算出每个 `acct_id` 值有多少次请求:

```bash
egrep -o 'acct_id=[0-9]+' access.log | 
	cut -d= -f2 | 
	sort | 
	uniq -c | 
	sort -rn
```

持续检测文件改动:

```bash
# 监控某文件夹中文件改变
watch -d -n 2 'ls -rtlh | tail'

# 排查 WiFi 设置故障时, 监控网络设置的任何更改:
watch -d -n 2 ifconfig
```

解析 Markdown 文件, 并随机抽取项目

```bash
function taocl() {
	curl -s $myurl |
		pandoc -f markdown -t html |
		iconv -f 'utf-8' -t 'unicode' |
		xmlstarlet fo --html --dropdtd |
		xmlstarlet sel -t -v "(html/body/ul/li[count(p)>0])[$RANDOM mod last()+1]" |
		xmlstarlet unesc | fmt -80
}
```

递归地删除日志文件:

```bash
fint . -name "*.exe"  -type f -print -exec rm -rf {} \\;

ls | grep %*.log% | xargs rm
```

若删除了某文件, 用 `du` 检查内存未释放, 可以检查该文件是否被进程占用:

```bash
lsof | grep deleted | grep "filename-of-my-big-file"
```

和用户进行交互:

```sh
echo "Please enter your username:"
read DBUSERNAME
```

判断上个命令是否成功执行并返回:

```sh
[[ $? -eq 0 ]]

if (( $EUID == 0 )); then
	echo "Please do not run as root"
	exit
fi
```

## 杂

### SheBang

Linux 脚本文件第一行 `shebang` 用于指定程序的执行器:

```bash
#!/usr/bin/env bash
```

```bash
#!/usr/bin/python
```

### 大括号包裹代码

编写脚本时, 使用大括号包裹代码. 他人复制代码时, 可以避免复制不全, 而执行不完整的代码:

```bash
{
	#hello
}
```

大括号包括的代码不会影响变量作用域, 即不会像管道, `()` 一样创建临时上下文.

### 大括号提供文本扩展

`mv foo.{txt,pdf} ../` 同时移动两个文件

`cp somefile{,.bak}` <=> `cp somfile somfile.bak`

`mkdir -p test-{a,b,c}/subtest-{1,2,3}`

`{1..10}` <=> `1, 2, 3, 4, 5, 6, 7, 8, 9, 10`

`{a..j}` <=> `a b c d e f g h i j`

`{1..10..2}` <=> `1 3 5 7 9`

`echo {a..c}{1..3}` <=> `a1 a2 a3 b1 b2 b3 c1 c2 c3`

### 跳脱符 \

`\`可以跳脱下一个输入的字符, 也就是下一个字符输入后不再表达原本意义. 

常用于`\[Enter]`,让[Enter]不再是执行当前行命令, 而是单纯**换行**

### 命令历史

`history [n]` 显示 (过去 n 个) 编号历史

`history -c` 清除历史

`echo ${HISTSIZE}$`

`!!` 上一条命令

`!n` 执行第 n 条历史命令 (用 `^R` 更方便)

`!$$` 上一条参数


### [128K限制](https://wiki.debian.org/CommonErrorMessages/ArgumentListTooLong)

通配符匹配大量文件名时, 遇到 "Argument list too long" 的错误信息.

比如直接重定向会报错:
```bash
find . type f -name "*.txt" | grep "pattern"
```

可以使用 `xargs` 或 `find -exec` 解决:

```bash
find . -type f -name "*.txt" -exec grep "pattern" {} +

find . -type f -name "*.txt" -print0 | xargs -0 grep "pattern"
```

## 调试

> 调试代码比写代码困难两倍, 写代码用越多奇技淫巧 (自作聪明), 调试代码就越痛苦

### bash 调试参数

```bash
bash -n script # 不执行, 仅检查一遍 syntax errors
bash -v script # 打印每条执行
bash -x script # 打印每条执行的指令, 该指令非原始形式, 而是已经过命令行解析

# use with shebang:
#!/bin/bash -x
```

### 内部调试命令

```bash
	set -u # 检查变量 Use-Before-Init
	set -euo pipefail # 检查管道错误
	# 使用 trap 捕获错误
	trap "echo 'error: Script failed: see failed command above'" ERR 
```

### 捕获信号和返回值

被保留的进程返回值:

| 0     | normal                         |                                    | 错误例子              |
| ----- | ------------------------------ | ---------------------------------- | ----------------- |
| 1     | general erros                  | 除零等致命错误                     | `let "var=1/0"`   |
| 2     | misuse of (bash)shell builtins | 遗漏参数, 权限问题等               | `empty_func() {}` |
| 126   | command invoked not executed   | 不是可执行文件                     | `$ /dev/null`     |
| 127   | command not found              | 一般是 `PATH` 问题找不到可执行文件 | `illegal_cmd`     |
| 128   | invalid argus of exit          | 返回值只能是整数                   | `exit 3.15`       |
| 128+n | fatal error signal n           | `$?` 的返回值                      | `kill -9 $PPID`   |
| 130   | terminated by Ctrl-C           | fatal error signal 2 (130=128+2)   | Ctrl-C            |
| 255   | exit status out of range       | exit 只接收 0-255                  | `exit -1`                  |

其他退出值用户可以自定义, 不过实际标准很混乱. C 为不同错误规定了一些标准返回值, 见 `/usr/include/sysexits.h`

```bash
funciton cleanup {
	echo "exit..."
	exit 1
}
trap cleanup INT
```

## 控制流配置

### 函数分支

bash 中使用函数就像使用其他命令一样. 参数默认是值传递.

```bash
function greet() {
    echo "hello, world"
    return 0
}

greet | cat

# function 输出共有: sdtout, stderr, exit codes.
```

参数传递 != 标准输入

```bash
# 不要搞混 `标准输入` 和 `参数`, 两回事.
function say {
    echo $1
}
say "hello world!"
echo "hello, world!" | say # 这样是错误的, 

function say {
	read input # 用 read 接收标准输入, 或者使用 xargs 将标准输入或字符串转化为参数.
}
```

通过变量名传递间接引用:

```bash
foo() {
	local var=$1
	eval "$var='new value'"
}

var="hello"
foo "var"
echo $var 
>> nwe value

# bash 还呢个通过 ${!var} 来间接引用.
arr["key"]="value"
keyname="key"
echo "${!keyname}
```

### 条件分支

`[]` 是 `test` 命令的语法糖. `[[]]` 是 Bash 引入的对 `test` 命令的扩展. 推荐使用 `[[]]`, 功能多, 且不需要转义元字符.

```bash
# 注意1 [ ] 是关键字, 需要左右空格.
# 注意2 ; 符号是为了分割同行的命令, 视情况可以省略.
if [ expression ]; then
    ...
else
    ...
fi

if [ ... ] then echo "hello"; fi  # OK
if [ ... ]; then echo "hello"; fi # OK
```

switch 语句:

```bash
case expression in
    pattern1 )
        statements ;;
    pattern2 )
        statements ;;
    ...
esac
```

`[]` 与 `[[]]` 的区别:

```sh
# [] 需要 "" 预防单词拆分
[ "$str1" = "$str2" ]

# [[ ]] 不需要引号
[[ $str1 == $str2 ]]


# [[ ]] 使用 &&, ||, == 更符合编程习惯
[ "$var1" -eq 1 -a "$var2" -eq 2 ]
[[ $var -eq 1 && $var2 -eq 2 ]]

# [[]] 支持模式匹配
[[ $var == a* ]]

# [[]] 命令替换时无需引号
[[ $(command) == ".." ]]
```

更多语法:

```sh
statement1 && statement2 
statement1 || statement2

str1 == str2       # 字符串匹配, [] 中使用 =
str1 != str2      # 不匹配
str1 < str2       # str1 is less than str2
str1 > str2       # str1 is greater than str2
-n str1         # str1 is not null (has length greater than 0)
-z str1         # str1 is null (has length 0)

-a file         # 文件存在
-e file         # 和 -a 相同
-d file         # 文件存在, 且是文件夹
-f file         # 文件存在, 且是文本文件 (非文件夹, 或其他类型)
-L file         # 文件存在, 且是符号链接
-r file         # 有读权限
-s file         # 文件存在, 且非空
-w file         # 有写权限
-x file         # 有执行权限; 若是文件夹, 则有搜索权限
-N file         # 上次被读以来, 该文件已被修改.
-O file         # 属于我的文件
-G file         # 文件的 group ID 是我所在的群组.

file1 -nt file2     # file1 is newer than file2
file1 -ot file2     # file1 is older than file2

# 数值比较
-lt     # less than
-le     # less than or equal
-eq     # equal, 字符串比较应使用 ==
-ge     # greater than or equal
-gt     # greater than
-ne     # not equal
```

### 循环分支

`for` 类型

```bash
for x := 1 to 10 do
begin
  statements
end

# python style
for user in  ${users}
do
	echo "${user}"
done

# c style
for (( initialisation ; ending condition ; update ))
do
  statements...
done


for num in {1..10}
do 
	echo ${num}
done
```

`while` 类型

```bash
while condition; do
  statements
done

while command; # while 检查命令返回状态, 为 0 (正确退出)时, 执行; 否则退出.
while [ ]; # [ ] 是一个bash命令, 用于判断条件是否成立
```

`until` 类型: 相当于 `while !`

```bash
until condition; do
  statements
done
```

`continue`

`break 2` 跳出两层循环

#### `seq`

生成一个数字序列, 通常用于生成特定循环区间:

```bash
seq First [Step] Last

# 生成 3\n4\n5
seq 3 5

# 生成 3\n5
seq 3 2 5
```