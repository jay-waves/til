## 在终端模拟器中按 ESC 后有延时

终端模拟器, 如 xterm, alacritty, gnome-terminal 等, 与实际终端不同. 不过它们都使用**转义序列**来控制光标移动和改变文本颜色等特殊行为. 终端 ANSI 转义序列以 `ESC[` 开头 (一般显示为 `^[[` ), C/C++ 编程语言中则形如 `\r`.

```vim
:nnoremap ,a :echo 'foo'<cr>
:nnoremap ,ab :echo 'bar'<cr>
```

上面的例子中两个映射都能正常工作, 但是当输入 `,a` 之后, Vim 会延时 1 秒, 因为它要确认用户是否还要输入那个 `b`.

转义序列会产生同样的问题: 

- `<esc>` 作为返回普通模式或取消某个动作的按键而被大量使用
- 光标键使用转义序列进行的编码
- Vim 期望 <kbd>Alt</kbd> (也叫作 _Mate Key_ ) 会发送一个正确的 8-bit 编码的高位, 但是许多终端模拟器并不支持这个, 而只是发送一个转义序列作为代替.

简而言之, Vim 在区分录入 `<esc>` 和转义字符的时候需要一定延迟.

默认情况下, Vim 用 `:set timeout timeoutlen=1000`, 就是说它会用 1 秒的时间来区分有歧义的映射 _以及_ 按键编码. 这对于映射来说是一个比较合理的值, 但是可自定义:

```vim
set timeout           " for mappings
set timeoutlen=1000   " default value
set ttimeout          " for key codes
set ttimeoutlen=10    " unnoticeable small value
```

> 帮助 `:h ttimeout`