## 会话

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

标签页 (Tab Pages) 是一组窗口的集合. VIM 会话启动时, 有一个默认标签页和一个窗口. 当标签页不分屏时, 一个标签页就只有一个窗口; 当标签页分屏时, 就有了多个窗口.

```vim
:tabnew          " 打开一个新标签页
:tabn            " 切换到下一个标签页
:tabp
:tabc            " 关闭标签页
:tabs            " 列出所有标签页
:tabe <file>     " 在新标签页编辑文件
```

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
```

> 查看帮助: `:h CTRL-W_*`, 如 `:h CTRL-W_p` 查看切换窗口命令  
> 访问 `:h windows.txt` 查看所有窗口管理命令.

## 操作

**动作** -> 移动光标, 如 `h, j, k, l, w, b`

**操作符, operator** -> 对某区域执行操作, 如 `d, ~, gU, >`

**文本对象操作, text-objects** -> 用于选中区域 (如括号, 单词, 句子), 形式为 `i` (inner), `a`(around) 加上对象标识符. 如 `diw` 删除当前单词, `da(` 删除当前括号及其中内容.

normal mode 下, 应先输入操作符, 再输入动作. 如 `>j`

visual mode 下, 先选中区域, 然后按操作符

`d2a(` 和 `2da(` 等价, `4da(` 和 `2d2a(` 等价.

> `:h navigation`, `:h operator`, `:h text-objects`


## mode

```
normal mode -i,a-> insert mode
insert mode -esc-> normal mode
normal mode -v,V-> visual mode
normal mode -:-> command-line mode
```

****
Vim 总是区分 **行操作** 和 **字符操作**

以行为单位进行操作, 行也基本是最大的操作单位. 更大操作则需要先"选中".

字符操作使用小写字母, 行操作则是对应字符操作的大写字母.

字母g则对应全局操作, 用于命令增强.