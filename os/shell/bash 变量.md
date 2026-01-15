bash 变量没有严格数据类型, 默认都是字符串类型. 注意整型和布尔型都容易误用.
- 变量引用 `${var}`, 执行变量展开.
- 命令引用 `$(instru)`, 执行命令替换.

## 变量

- **环境变量**在任何子程序中都能使用, 有专门的继承内存块
- **自订变量**在当前父程序的子程序中无法使用

常见环境变量: 

- `PATH` 执行环境
- `PS1` 命令提示符
- `BASHPID` 当前进程的 PID, 仅在主脚本中, `$` 和 `BASHPID` 相同.
- `$` 顶层 Bash 脚本的 PID, Bash 中子脚本会直接继承.
- `PPID` 父进程的 PID. Bash 中子脚本会直接继承.
- `?` 上一个指令所传回的错误码
- `!` 上一条命令的PID
- `0` 当前脚本名称
- `1, 2, ..., 9, {10}, {11}` 位置参数, 代表传入的参数
- `#` 传入参数数量
- `IFS` Internal Field Separator, default like: `\s, \t, \n`
- `PWD` 工作目录
- `HOME` 家目录
- `UID, USER, HOSTNAME` 用户ID, 用户名, 主机名
- `RANDOM` 返回一个随机数
- `LINENO` 当前脚本执行到的行数, 常用于调试

### `export`

展示或设置环境变量. 系统级环境变量一般使用*大写字母*.

```bash
$ export
AWS_HOME=/Users/adnanadnan/.aws
LANG=en_US.UTF-8
LC_CTYPE=en_US.UTF-8
LESS=-R

$ echo $AWS_HOME
/Users/adnanadnan/.aws

# 将 TimeZone 设置斐济时间.
$ export $TZ=Pacific/Fiji data 

# 安全原因, 不要将相对路径 (如 `.`) 放入 PATH.
$ export PATH="${PATH}:\root"
```

### `env`

提供对临时环境变量更高级的支持. 

```bash
# 以 VAR1, VAR2 环境变量临时执行某命令
env VAR1=value1 VAR2=value2 command
# 当然这样也可以
VAR1=value1 VAR2=value2 command

# 清除所有环境变量, 包括 export 设置的变量
env -i command 
```

## 数组

Examples:

```bash
array[0]=val
array[1]=val
array[2]=val
array=([2]=val [0]=val [1]=val)
array=(val val val)
```

取址访问:

```bash
${array[i]}     # where i is the index
```

获取当前数组大小:

```bash
${#array[@]}
```

变量三元操作:
```bash
# 如果 str 不存在, 返回 expr; 否则返回 str.
${str-expr}
# 如果 str 不存在或为空 (""), 返回 expr; 否则返回 str.
# `:` 添加了对空 `""` 的检查.
${str:-expr}
# 如果 str 已经存在, 返回 str, 否则令 str=expr, 然后返回 str.
${str=expr}
# 如果 str 不存在(未初始化), 将 expr 输出到 stderr.
${str?expr}
# performs substring expansion. It returns the substring of $varname 
# starting at offset and up to length characters
${varname:offset:length}    
```

## 字符串

`""` 和 `''` 都用于定义字符串, 但 `''` 不进行任何变量展开(interpolation)和命令替换(command substitution), 纯字面量. 字符串皆支持直接跨行输入.

三元匹配操作:

```bash
# 从头匹配 pattern, 找到则删除
${variable#pattern}         
# 从头贪婪(尽可能长)匹配 pattern, 找到则删除
${variable##pattern}        
# 从末尾匹配 pattern, 找到则删除
${variable%pattern}         
# 从末尾贪婪匹配 pattern, 找到则删除
${variable%%pattern}        
# 用 string 替换贪婪匹配到的第一个 pattern
${variable/pattern/string}  
# 用 string 替换贪婪匹配到的所有 pattern
${variable//pattern/string} 
# 返回数组长度, 即字符串字符数量
${#varname}     
```

大小写转换:

```bash
# converts every letter in the variable to lowercase
${variable,,}  
# converts every letter in the variable to uppercase
${variable^^}    
```

索引:
```bash
# this returns a substring of a string, starting at the character at 
# the 2 index(strings start at index 0, so this is the 3rd character),
# the substring will be 8 characters long, so this would return a 
# string made of the 3rd to the 11th characters.
${variable:2:8}    
```

匹配字符串首尾:

```bash
#this returns true if the provided substring is *in* the variable
if [[ "$variable" == *subString* ]]  
#this returns true if the provided substring is not in the variable
if [[ "$variable" != *subString* ]]  
#this returns true if the variable starts with the given subString
if [[ "$variable" == subString* ]]   
#this returns true if the variable ends with the given subString
if [[ "$variable" == *subString ]]   
```

使用模式匹配简化匹配操作:

```bash
case "$var" in
    begin*)
        #variable begins with "begin"
    ;;
    *subString*)
        #subString is in variable
    ;;

    *otherSubString*)
        #otherSubString is in variable
    ;;
esac
```
