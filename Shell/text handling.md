## 文件浏览和拼接

### `echo` & `printf`

```sh
$ echo -ne "Hello\nWorld\n"
Hello
World
```

`printf` 类似 `echo -n`:

```sh
printf '%s\n' 'hello world'
```

**注意, `echo` 不接受文件输入或标准输入, 如 `echo < hello.txt` 是无效的. 它仅重复输入的参数, 其他情况应使用 `cat`**

### `cat`
  
  ```bash
  cat file1 file2 
  cat file1 file2 > combinedfile
  cat < file1 > file2           # copy file1 to file2
  cat > file                    # accept input from keyboard
  ```

更现代的工具: `bat`

### `diff`

Compares files, and lists their differences.  

```bash
diff filename1 filename2
```

标准的源代码对比及合并工具是 `diff` 和 `patch`. 使用 `diffstat` 查看变更总览数据. 注意到 `diff -r` 对整个文件夹有效. 使用 `diff -r tree1 tree2 | diffstat` 查看变更的统计数据. `vim diff` 用于比对并编辑文件. 

### `tail`

Outputs the last 10 lines of file. Use `-f` to output appended data as the file grows.  

```bash
tail filename
```

### `head`

Outputs the first 10 lines of file  

```bash
head filename
```

### `more`, `less`

Shows the first part of a file (move with space and type q to quit).  

```bash
more filename
```

### `split`, `csplit`

使用 `split` (按大小拆分) 和 `csplit` (按模式拆分) 拆分文件.

### `cut`, `paste`, `join`

Remove sections from each line of files

*example.txt*

```bash
red riding hood went to the park to play
```

*show me columns 2 , 7 , and 9 with a space as a separator*

```bash
cut -d " " -f2,7,9 example.txt
```

```bash
riding park play
```

## 文件内容过滤

### `awk`

awk is the most useful command for handling text files. It operates on an entire file line by line. By default it uses whitespace to separate the fields. The most common syntax for awk command is

```bash
awk '/search_pattern/ { action_to_take_if_pattern_matches; }' file_to_parse
```

Lets take following file `/etc/passwd`. Here's the sample data that this file contains:

```
root:x:0:0:root:/root:/usr/bin/zsh
daemon:x:1:1:daemon:/usr/sbin:/usr/sbin/nologin
bin:x:2:2:bin:/bin:/usr/sbin/nologin
sys:x:3:3:sys:/dev:/usr/sbin/nologin
sync:x:4:65534:sync:/bin:/bin/sync
```

So now lets get only username from this file. Where `-F` specifies that on which base we are going to separate the fields. In our case it's `:`. `{ print $1 }` means print out the first matching field.

```bash
awk -F':' '{ print $1 }' /etc/passwd
```

After running the above command you will get following output.

```
root
daemon
bin
sys
sync
```

For more detail on how to use `awk`, check following [link](https://www.cyberciti.biz/faq/bash-scripting-using-awk).

### `sed`

Stream editor for filtering and transforming text

*example.txt*

```bash
Hello This is a Test 1 2 3 4
```

*replace all spaces with hyphens*

```bash
sed 's/ /-/g' example.txt
```

```bash
Hello-This-is-a-Test-1-2-3-4
```

*replace all digits with "d"*

```bash
sed 's/[0-9]/d/g' example.txt
```

```bash
Hello This is a Test d d d d
```
 
 > [详见](https://www.cnblogs.com/liwei0526vip/p/5644163.html)

### `grep`

```bash
$ grep admin /etc/passwd
_kadmin_admin:*:218:-2:Kerberos Admin Service:/var/empty:/usr/bin/false
_kadmin_changepw:*:219:-2:Kerberos Change Password Service:/var/empty:/usr/bin/false
_krb_kadmin:*:231:-2:Open Directory Kerberos Admin Service:/var/empty:/usr/bin/false
```

force grep to ignore word case by using `-i` option. `-r` can be used to search all files under the specified directory:

```bash
$ grep -r admin /etc/
```

`-F`, `-E` enable functional of regular expr.

`-C [N]` context: print `N` lines after and before matching lines:
```bash
grep -C 5 "pattern"
```


***

## 格式化工具

### `fmt`

Simple optimal text formatter

*example: example.txt (1 line)*

```bash
Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.
```

*output the lines of example.txt to 20 character width*

```bash
cat example.txt | fmt -w 20
```

```bash
Lorem ipsum
dolor sit amet,
consetetur
sadipscing elitr,
sed diam nonumy
eirmod tempor
invidunt ut labore
et dolore magna
aliquyam erat, sed
diam voluptua. At
vero eos et
accusam et justo
duo dolores et ea
rebum. Stet clita
kasd gubergren,
no sea takimata
sanctus est Lorem
ipsum dolor sit
amet.
```

### `uniq`, `sort`

Report or omit repeated lines

*example.txt*

```bash
a
a
b
a
b
c
d
c
```

*show only unique lines of example.txt (first you need to sort it, otherwise it won't see the overlap)*

```bash
sort example.txt | uniq
```

```bash
a
b
c
d
```

*show the unique items for each line, and tell me how many instances it found*

```bash
sort example.txt | uniq -c
```

```bash
    3 a
    2 b
    2 c
    1 d
```

### `nl`

number lines, 打印行号, `nl file1`