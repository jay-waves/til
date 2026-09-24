### nvim on windows 

配置放在 `~/AppData/Local/nvim/init.lua` 

从 nvim 0.12.0 开始，直接用内置包管理器即可。插件不用太多：
* 主题: github-nvim-theme 
* 状态栏、Tab 栏: lualine.nvim 
* 语法解析（高亮）：nvim-treesitter
* GIT 状态、差异对比： gitsigns 
* 补全：blink.cmp 
* find, grep 集成：fzf-lua 
* 文件树：mini.files 

详细配置详见 [jay-waves/dotfiels](https://github.com/jay-waves/dotfiles)

### keymaps 

* `K` hover -> document 
* `gD` 
* `gd` 
* `<leader>l`, `<leader>L` : diagnostics 
* `<C-space>` : blink.cmp menu toggle 
* `<C-LeftMouse>` : jump to file `gf`
* `<leader>p` FzfLua 
* `<leader>b` FzfLua buffers 
* `<leader>O` FzfLua lsp_document_symbols / treesitter symbols
* `<leader>f` format selected buffer (lsp)

