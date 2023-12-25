## 提示

- 示例:

`ls --help` 
`man ls`
`info ls`
`help cd`

`man`

类似文档, 例子少, 很难懂但很全. 个人更喜欢info. `man section search_item`  
section是数字1-8, 指的是八种不同的文档关键词分类  
`man -k`相当于`apropos`, 可以进行关键词搜索 

`whatis`

显示单行, 非常简短的命令介绍


`type ``

`type command` 显示某个命令的类型, linux中命令有四种类型:
- /bin文件夹中的可执行程序
- shell builtins内置程序
- shell function即scripts
- alias 别名

`which`

`which ExeCommand` 显示所执行程序的具体目录位置. 只能检索环境变量PATH中的可执行程序, 不能检索其他类型命令(如别名/内置程序都会报错). 用于检查当存在两个同名程序(比如同一程序的不同版本)时, 具体执行了哪个程序.

## 命令昵称

`alias`

注意, bash一行可以有多个命令, 每个命令用`;`隔开.

- Format: `$ alias new_expr=old_instru_expr`

```bash
e.g.
$ alias lm='ls -al | more'
```

- Format: `$ alias`
show all the current alias

- Format: `$ unalias old_instru_alias`

- Format `$ history`
1. show the instru history (up to 1000?, use `$ history [n]` to show the lasted n instru)
2. other parameters: 
	- `[-c]` clear the history
	- `$history [-raw] histfiles`, default histfile is `~/.bash_history`, -a ->append; -r->read (the content of histfiles); -w -> write
3. `$ echo ${HISTSIZE}`

- Format `$ !no/cmd/!`
1. !! last cmd