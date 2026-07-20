
Bash 执行命令时，可能有以下输入： 
- `argv[]` 字符串，作为 `c main` **参数**输入
- `stdin` ，可能来自文件，也可能来自终端用户输入

有如下输出：`stdout, stderr`

## 重定向

### stdout 重定向

覆盖写入
```bash
echo "Hello, World!" > output.txt

# 丢弃所有标准输入
echo "..." > /dev/null
```

附加写入, 而不是覆盖
```bash
echo "hello, world!" >> output.txt
```

注意 `echo` 不接受 `stdin` 输入，只将参数解析为字符串。而 `cat` 将参数作为文件读取，因此可接收
`stdin` 输入。

### stderr 重定向

```bash
ls not_existing_file 2> error.log
ls not_existing_file 2>> error.log
```

同时重定向 stderr + stdout:

```bash
ls not_exisiting_file &>> output.log

# 将 stderr 重定向至 stdout 的地址
cat not_existed_file >logfile 2>&1
```

### stdin 重定向

echo 命令默认将参数


```bash 
sort < unsorted.txt

# 关闭标准输入, 立即返回 EOF, 用于确保后台命令不再期望读入输入, 
# 而立即收到 EOF 关闭句柄
cmd < /dev/null

# <<< 将字符串直接作为命令的标准输入
grep "pattern" <<< "This is search term in string"
```

### here documents

bash 中一种重定向标准输入方法, 可以在命令行直接提供多个行的输入.

```bash
kubectl apply -f - <<EOF
apiVersion: v1
kind: Pod
metadata:
  name: mypod
  labels:
    app: myapp
spec:
  containers:
  - name: mycontainer
    image: nginx
    ports:
    - containerPort: 80
EOF
```

```bash
cat << my-end-flag > lab.cc
#include <stdio.h>
static long num_steps = 100000000;
double step, pi;
int main() {
	...
	return 0;
}
my-end-flag
```

"here string" 类似, 将字符串重定向到 stdin.

```bash
grep hello <<< "hello world"
```

等价于： 

```bash
echo "hello world" | grep hello
```

## 管道

管道 `|` 底层使用 Unix pipe 系统调用, sh 在两个命令之间建立一个缓冲区, 存储前一个命令的标准输出.

```bash
# 统计行数
find . -type f -name "*.txt" | wc -l
# 排序并去重
cat file.txt | sort | uniq
```

每个管道都是在**局部环境**中执行, 变量也都是局部变量(即子shell), 仅有 stdout 被父进程捕获.

```bash
mkfifo /tmp/myfifo
while read line < /tmp/myfifo; do
	echo "阻塞读取 fifo"
done
```

### `tee`

用于保留一份 stdout 的副本, 同时不干扰正常 stdout 输出.

```bash
# 保留 stdout 副本到多个文件, 同时屏幕仍有 cmd1 输出
cmd1 | tee file1 file2 file3

# `cmd1 | cmd2`, 同时将 cmd1 输出附加到 file1 中
cmd1 | tee -a file1 | cmd2

# 修改 /etc/ocnfig.conf, 同时输出修改内容
echo "Some configuration" | sudo tee /etc/someconfig.conf
```

## `xargs`

`xargs` 可以将字符串或 stdin 转化为命令参数:

```bash
echo "on two three" | xargs mkdir
# ls: ./on  ./two ./three
```

删除 `find` 命令找到的文件:

```bash
find . -name "*.txt" -type f | xargs rm
```


`-0` + `find ... --null`: 当文件名中可能包含空格或特殊字符时, 使用 `NULL` 隔开不同项, 而不是空格. 如 `fd, xargs` 支持用参数 `-0` 开启该功能, `find` 则使用参数 `-print0`

```bash
find . -name "*.txt" -type f -print0 | xargs -0 rm
```

**Bash 默认的分隔符（IFS，Internal Field Separator）包括 `\s, \t, \n`，可以修改**。

