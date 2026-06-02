<https://wxamples.com>

## Editor Plugins

- Advanced Tables
- Reveal Active File Button, 在文件管理器高亮当前文件
- Table of Contents, MD 目录生成器
- Tag Wrangler, 标签管理器
- Recent Files, 查看最近打开文件
- ~~Iconize, 给文件夹和文件加图标~~
- Dangling links, 检查是否有无效链接
- ~~Regex Pipeline~~, 用正则表达式格式化文本的, 下面两个例子

```regex
:: 将 \[\] \(\) 数学公式符号替换为 $ $ 格式
"\\\[\s*([\s\S]*?)\s*\\\]"->"$@@@$@@@$1$@@@$"
:: \s* 用来去除两端的空格, 避免出现替换为 $ xxx $ 的无效格式
:: 使用 $1 引用捕获内容, 但是会和数学公式的符号重叠, 插入一些 @@@ 来二次替换
"\\\(\s*(.*?)\s*\\\)"->"$@@@$1$"
"@@@"->""

:: 用半角加空格替代全角符号
" "->" "  
"。"->". "  
"，"->", "  
"；"->"; "  
"："->": "  
"？"->"? "  
"！"->"! "  
"（"->" ("  
"）"->") "
```

- ~~Linter~~, 也是格式化工具. 如自动将半角和全角符号间加上空格. 似乎有不少 BUG 
- Obsidian Git, 很喜欢 diff view 和 source control view.
- Ninja Cursor. 光标闪烁，提高可读性
- Codeblock Customizer. 嵌入 PrismJS 的代码块渲染，很现代。
- Advanced Search UI. 增强默认搜索界面

## Vim plugins

- Cursor Location, 状态栏查看当前光标的位置 (行列), 搭配 Vim 使用的.
- [Vim IM Select](vim/FAQ/多语言输入.md) / Vimrc Support
- Vim IM Control
- Word Splitting for Simplified Chinese in Edit Mode and Vim Mode

## Math Plugins

- Completr, 弹窗提示补全
- Latex Suites
- Quick Latex for Obsidian
- No more flickering inline math, 注意和 Latex Suites 的兼容问题
- KaTex to MathJax. 把 AI 喜欢的 `\(\)` 风格改为传统 `$$` 风格
- Extended MathJax. 允许预定义一些 Latex 数学函数或命令