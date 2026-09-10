### nvim on windows 

配置放在 `~/AppData/Local/nvim/init.lua` 

从 nvim 0.12.0 开始，直接用内置包管理器即可。插件不用太多：
* 主题: github-nvim-theme 
* 状态栏、Tab 栏: lualine.nvim 
* 语法解析（高亮）：nvim-treesitter
* GIT 状态、差异对比： gitsigns 
* 补全：blink.cmp 
* find, grep 集成：fzf-lua 

不太喜欢 nvim 内部的文件树插件，我主要用：
* `:e .` 内置的文件选择器 
* [`lf`](https://github.com/gokcehan/lf) 命令行文件管理器，GO 写的，很轻量好用。

详细配置详见 [jay-waves/dotfiels](https://github.com/jay-waves/dotfiles)

### lsp 

lsp code jump:
* `grt` type definition 
* `gri` implementation 
* `grr` references
* `gra` code actions 
* `grn` rename 
* `K`   hover (documentation)
* `Ctrl-]` definition (gvim)
