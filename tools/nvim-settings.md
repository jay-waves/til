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

```lua
vim.opt.number = true
vim.opt.tabstop = 4
vim.opt.shiftwidth = 4
vim.opt.cursorline = true
vim.opt.smartindent = true
vim.opt.showcmd = true
vim.opt.wrap = false
vim.opt.ignorecase = true
vim.opt.smartcase = true
vim.opt.incsearch = true
vim.opt.hlsearch = true
vim.opt.termguicolors = true
vim.opt.laststatus = 2

vim.keymap.set("n", "j", "gj", { noremap = true })
vim.keymap.set("n", "k", "gk", { noremap = true })


vim.api.nvim_create_autocmd("PackChanged", {
  callback = function(ev)
    local name = ev.data.spec.name
    local kind = ev.data.kind

    if name == "nvim-treesitter"
      and (kind == "install" or kind == "update")
    then
      if not ev.data.active then
        vim.cmd.packadd("nvim-treesitter")
      end

      vim.cmd.TSUpdate()
    end


  end,
})

vim.pack.add({
  "https://github.com/projekt0n/github-nvim-theme",
  "https://github.com/nvim-lualine/lualine.nvim",
  "https://github.com/nvim-treesitter/nvim-treesitter",
  "https://github.com/lewis6991/gitsigns.nvim",
  "https://github.com/ibhagwan/fzf-lua",

  { src = "https://github.com/saghen/blink.cmp", version = "v1", },
})


require("github-theme").setup({ options = { transparent = true, }, })

vim.cmd.colorscheme("github_dark_tritanopia")

require("lualine").setup({
  options = {
    theme = "auto", 
    icons_enabled = true,
    globalstatus = true,
  },

  sections = {
    lualine_a = { "mode" },
    lualine_c = { { "filename", path = 1, } },
    lualine_x = { "filetype" },
    lualine_y = { "progress" },
    -- lualine_z = { "location" },
  },

  tabline = {
    lualine_a = {
      {
        "buffers",
        mode = 0, 
        show_filename_only = true,
        show_modified_status = true,
		max_length = function()
			return math.floor(vim.o.columns * 0.9)
		end,
      }
    },
  },
})

require("gitsigns").setup({})
vim.api.nvim_create_user_command("GitPreview",
	function()
		require("gitsigns").preview_hunk()
	end,
{})
vim.api.nvim_create_user_command("GitNext",
  function()
    require("gitsigns").next_hunk()
  end,
{})
vim.api.nvim_create_user_command("GitPrev",
  function()
    require("gitsigns").prev_hunk()
  end,
{})
vim.api.nvim_create_user_command("GitBlame",
  function()
    require("gitsigns").blame_line()
  end,
{})

require("nvim-treesitter").install({
  "lua",
  "vim",
  "vimdoc",
  "query",
  "markdown",
  "markdown_inline",
  "bash",
  "python",
  "javascript",
  "typescript",
  "tsx",
  "html",
  "xml",
  "css",
  "json",
  "yaml",
  "go",
  "rust",
  "c",
  "cpp",
  "typst"
})

vim.api.nvim_create_autocmd("FileType", {
  callback = function()
    pcall(vim.treesitter.start)
  end,
})


vim.api.nvim_create_autocmd("VimEnter", {
  once = true,
  callback = function()
    local ok, blink = pcall(require, "blink.cmp")
    if not ok then
      vim.notify("blink.cmp not loaded", vim.log.levels.WARN)
      return
    end

    blink.setup({
      keymap = { preset = "super-tab", },

      sources = {
        default = {
          "path",
          "buffer",
		  "lsp",
        },
      },

      completion = {
        documentation = { auto_show = false, },
        ghost_text = { enabled = true, },
      },
    })
  end,
})

```


### nvim on wsl debian 

放在 `~/.config/nvim/init.vim` 

```vim
set number                                      
set tabstop=2                                   
set shiftwidth=2                                
set expandtab                                   
set cursorline                                 
set smartindent  
set showcmd
" set wrap                                        

" search
set ignorecase                                  
set hlsearch
set smartcase                                  

set mouse=a

" clipboard
set clipboard=unnamedplus

" encoding
set encoding=utf-8
set fileencoding=utf-8

" plugins
call plug#begin('~/.local/share/nvim/plugged')
Plug 'catppuccin/nvim', {'as': 'catppuccin'}
Plug 'vim-syntastic/syntastic'                  " 语法检查插件
Plug 'vim-airline/vim-airline'                  " taskbar美化插件
Plug 'vim-airline/vim-airline-themes'
Plug 'ctrlpvim/ctrlp.vim'                       " 模糊查找文件
call plug#end()

" theme
syntax on
set termguicolors
colorscheme catppuccin-macchiato 
let g:airline_theme="catppuccin" "latte, frappe, macchiato, mocha

" 背景透明，如果 terminal 带透明效果，这个非常重要
hi Normal ctermbg=none guibg=none
hi NonText ctermbg=none guibg=none

" keyboard
nmap j gj
nmap k gk

" taskbar
set t_Co=256                                    "色域设置
set laststatus=2                                 
let g:airline_powerline_fonts = 1   
let g:airline#extensions#tabline#enabled = 1
let g:airline_section_c = ''                    " 禁用文件名
```
