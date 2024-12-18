## 会话

### 缓冲区

缓冲区 (Buffers) 是文件在内存中的表示, 文件被打开后存储到**全局的缓冲区列表**中, 所有标签页或窗口共享该列表. *不用多标签页时, VIM 最上方的栏其实是缓冲区列表.*

```vim
:bn           " 切换到下一个缓冲区, buffer next
:bp           " 切换到上一个缓冲区, buffer previous
:b2           " 切换到第二个标签页, 用 :buffers 查看编号
:bd <buffer>  " 删除缓冲区, buffer delete
:ls
:buffers      " 列出全局缓冲区列表
:e <file>     " 激活新 buffer, 隐藏当前 buffer
```

### 标签页

标签页 (Tab Pages) 是一组窗口的集合. VIM 会话启动时, 有一个默认标签页和一个窗口. 当标签页不分屏时, 一个标签页就只有一个窗口; 当标签页分屏时, 就有了多个窗口.

```vim
:tabnew          " 打开一个新标签页
:tabn            " 切换到下一个标签页
:tabp
:tabc            " 关闭标签页
:tabs            " 列出所有标签页
:tabe <file>     " 在新标签页编辑文件
```

### 窗口

窗口 (Windows) 是显示缓冲区的区域.

```vim
:sp  <file>           " 分屏
:vs  <file>           " 垂直分屏
:new <file>
^w w                  " 窗口间切换
^w h/j/k/l            " 在窗口间按方向键切换
^w H/L/x              " 窗口之间调换
^w c                  " 关闭分屏
:only
:q                    " 关闭当前窗口
:set (no)scrollbind   " 左右屏同时滚动

^w <, ^w <            " 修改左右侧分屏的宽度
^w +, ^w -            " 修改上下分屏的高度
^w =                  " 让分屏大小平均
```

> 查看帮助: `:h CTRL-W_*`, 如 `:h CTRL-W_p` 查看切换窗口命令  
> 访问 `:h windows.txt` 查看所有窗口管理命令.

### 视图

视图 (view) 是对当前窗口的快照, 会话 (session) 是对当前 Vim 工作区的整体快照. 存储当前窗口布局, 打开的缓冲区, 设置的命令, 按键映射等, 类似于"项目/工作区"的概念. 

```vim
:mkview my_view.vim             " 存储于当前文件夹下
:loadview my_view.vim

:mksession my_session.vim
:source my_session.vim
```

## 模式与操作

Vim 有三种类型操作:
- [移动](移动.md), action, 用于移动光标, 如 `h, j, k, l, w, b`
- [操作符](宏.md), operator, 用于对某区域执行操作, 如 `d, ~, gU, >`. 默认区域有**字符和行**, 字符操作用小写字母, 行操作用大小字母.
- **文本对象, text-objects**, 用于选中特殊区域 (如当前光标所在的括号, 单词, 句子), 形式为 `i` (inner), `a`(around) 加上对象标识符. 如 `diw` 删除当前单词, `di"` 删除引号中内容, `da(` 删除当前括号及其中内容. 

Vim 有三种操作模式用于不同目的:
- 普通模式 (Normal Mode), 先输入操作符, 再输入动作, 如 `>j`
- 插入模式 (Insert Mode), 用于输入文本.
- [选中模式 (Visual Mode)](范围与区域.md), 先选中区域, 然后按操作符.
- PS: 还有一种命令模式, 输入 `:` 后键入控制命令.

![|400](../../attach/Vim%20模式间切换.avif)

> 比如: `d2a(` 和 `2da(` 等价, `4da(` 和 `2d2a(` 等价.
>
> `:h navigation`, `:h operator`, `:h text-objects`

### 定义事件回调

快速跳转到头文件:

```vim
autocmd BufLeave *.{c,cpp} mark C
autocmd BufLeave *.h       mark H
在离开文件事件 (BufLeave) 发生时, 自动产生标记
```

手动触发事件:

```vim
:doautocmd BufRead
```

自定义事件:

```vim
doautocmd User Chibby

autocmd User Chibby call ChibbySetup()
```

## 临时工作文件

根据选项的不同, Vim 最多会创建 4 种工作文件. 

### 备份文件

`:set writebackup` 在写入前先备份文件, 当文件新的修改被保存时, 备份立即被删除.

`:set backup` 在写入前备份文件, 有修改保存时不删除备份. ( `:set nobackup` )

借助备份, 查看文件历史改动: (和临时历史不同)

```sh
$ diff ~/.vim/vimrc ~/.vim/files/backup/vimrc-vimbackup
390d389
< command! -bar -nargs=* -complete=help H helpgrep <args>
```

> 帮助见 `:h backup`

### 交换文件

编辑文件时, Vim 会创建一个临时交换文件, 里面是**当前未保存的所有修改**. 用 `:swapname` 获取交换文件路劲, 默认是在同文件夹下的隐藏文件, 以 `.file.swp` 结尾. 用 `set noswapfile` 来禁用交换文件.

断电时, 该文件并不会被删除. 提供了一定的容灾恢复, 服务器中不建议移动 `.swp` 的默认位置, 否则多个用户同时编辑同一文件时, 不会得到警告.

> 帮助见 `:h swap-file`, `:h usr_11`

### 撤销文件

内容变更历史记录是保存在内存中的, 并且会在 Vim 退出时清空. 如果想让它持久化到磁盘中, 可以设置 `:set undofile`. 这会把文件 `~/foo.c` 的撤销文件保存在 `~/foo.c.un~`.

> 帮助见 `:h 'undofile'` 和 `:h undo-persistence`

### viminfo 文件

备份文件, 交换文件和撤销文件都是与文本状态相关的. 而 viminfo 文件是用来保存在 Vim 退出时可能会丢失的其它辅助信息的, 包括: 历史记录 (命令历史, 搜索历史, 输入历史), 寄存器内容, 标注, 缓冲区列表, 全局变量等等.

默认情况下, viminfo 保存在 `~/.viminfo`, 非易失.

> 帮助见 `:h viminfo` 和 `:h 'viminfo'`


