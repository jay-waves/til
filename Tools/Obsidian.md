## 插件

- Admonition, 没啥用...
- Advanced Tables
- Reveal Active File Button, 在文件管理器高亮当前文件
- Table of Contents, MD目录生成器
- Tag Wrangler, 标签管理器
- Vim IM Select / Vimrc Support 
- Pandoc Plugin, 需要安装 pandoc
- Recent Files, 查看最近打开文件
- Editor Syntax Highlight, 代码高亮增强
- Iconize, 给文件夹和文件加图标
- Ninja Cursor, 光标闪烁
- Obsidian Git 
- Cursor Location, 状态栏查看当前光标的位置 (行列)
- Dangling links, 检查是否有无效链接
- Linter, 格式化 Markdown 工具
- Vim IM Control
- Word Splitting for Simplified Chinese in Edit Mode and Vim Mode
- Regex Pipeline, 用正则表达式格式化文本的, 下面两个例子

```regex
:: 将 \[\] \(\) 数学公式符号替换为 $ $ 格式
"\\\[\s*([\s\S]*?)\s*\\\]"->"$@@@$@@@$1$@@@$"
:: \s* 用来去除两端的空格, 避免出现替换为 $ xxx $ 的无效格式
:: 使用 $1 引用捕获内容, 但是会和数学公式的符号重叠, 插入一些 @@@ 来二次替换
"\\\(\s*(.*?)\s*\\\)"->"$@@@$1$"
"@@@"->""

:: 用半角替代全角符号
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

## 数学插件

- Completr, 弹窗提示补全
- Latex Suites
- Quick Latex for Obsidian
- No more flickering inline math, 注意和 Latex Suites 的兼容问题

## 文件结构

- ReadMe
- attach, 附件文件夹
- paper, 软链接到论文文件夹
- Glossary, 名词表
- appendix, 附录
- .git
- .obsidian

