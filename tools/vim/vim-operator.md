
Operators all interact with Vim’s built-in registers: operations first write content into registers, then other operations retrieve values from those registers.

## 选中, Visual Mode

- `v`
- `V` 以行为单位
- `^v` 以块(段落)为单位
- `o, O` 跳转至选中区域的开头/结尾
- `gv` 重选上次选中的文本
- `va(` 选中当前所在括号，如 `(xxx)`，类似地还有 `", ', (, {, [`

## 删除 `d`

Vim 删除等同于剪切，将文本从缓冲区删除，存储默认寄存器。

- `d` delete a text-object
- `x` delete a char 

## 修改 `c` 

- `c` 修改当前所在单词

## 复制 `y`

`y` 是一个动词, 使用方式为: 对象(寄存器, 选中, 区域) + `y` + 移动

1. 选中 + `y`
2. `y` + 移动, 如 `y3fa`, `y4l`, `0y$`
3. `yy` 整行, 含换行符. 实际是 `y_` 的别名.
4. `yW` 复制到下一个单词
5. `y'a` 复制到下一个含标签 `a` 的行
6. `y/foo` 复制到下一个含文字 `foo` 的行

## 行合并 `J`

1. 将光标放在第一行
2. 摁 `J` , 两行将合并
3. 多行合并, 可以选中后, 输入命令 `:join`

## 寄存器

- `:reg` 查看所有寄存器值
- `:<reg>` 查看某一寄存器值, 传送至管道
- `"<reg>[cmd]` 调用寄存器, 传入命令 cmd
- `q<reg>` 录制宏存储到寄存器

| 表示符号      | 名称                | 作用                                         |
| ------------- | ------------------- | -------------------------------------------- |
| `"`           | unnamed             | 默认, 缓存最后一次操作内容 (`d,c,s,x,y`)                   |
| `1-9`         | numbered            | 缓存最近操作内容, `"1-9` 缓存最近9次删除内容 |
| `0`           | copy                | 缓存最近一次复制的内容                       |
| `-`           | small delete        | 缓存行内删除内容                             |
| `a-zA-Z`      | named               | 用于主动指定                                 |
| `.`, `:`, `%` | read-only           | 缓存*最近命令*, 最近插入文本, 当前文件名     |
| `=`           | expression          | 用于执行表达式命令                           |
| `*`, `+`| selection | 存取GUI选择的文本; `"+` 为系统剪贴板          |
| `_`           | black hole          | 不缓存操作内容 (干净删除)                    |
| `/`           | last search pattern | 缓存最近的搜索模式                           |

非只读寄存器, 用户都有权修改其内容:

```vim
:let @/ = 'register'
n      " 跳转到下一个 register 出现.
```

向具名寄存器追加内容:

```vim
" 将当前行复制到寄存器 a
"aY   
  
" 将该行追加到寄存器 a, 使用 a 的大小模式 A 标识.
"AY 
```

