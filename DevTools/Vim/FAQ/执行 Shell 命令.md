开启内置终端:

```vim
:Terminal
:term

" 关闭
^w :quit!
```

执行 shell 命令: 结果显示在临时窗口.

```vim
:!my_command
```

执行命令, 并替换所选范围内的文本为命令输出: (范围也可以先用 `V` 选中, 而非指定)

```vim
:.,+4!ls
:$!ls                " 在文件末尾输出 ls 命令结果
```

执行命令, 并将输出添加到光标处:

```vim
:r !command
```

在普通模式, 连续输入 `!!` , 相当于执行 `:.!`

### 消息重定向

用 `:redir` 重定向*命令*的消息, 输出到某个文件, 寄存器或变量中.

```vim
:redir => neatvar
:reg                " 执行该命令, 并重定向
:redir END
:echo neatvar
:put=neatvar        " 输出变量内容到当前缓冲区

:put=execute('reg') " 上述等价于
```

> `:h :redir`