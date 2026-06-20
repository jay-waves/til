### nvim on windows 

配置放在 `~/AppData/Local/nvim/init.vim` 

```vim
set number                                      " 行号
set tabstop=4                                   " tab 宽度 
set shiftwidth=4                                " 缩进宽度
"set expandtab                                  " 将tab解释为空格
set cursorline                                  " 高亮当前行
set smartindent  
set showcmd
set nowrap                                        " 自动换行
set ignorecase                                  " 搜索时忽略大小写
set smartcase                                   " 如果搜索包含大写字符，则自动启用大小写敏感
set incsearch
set hlsearch
set showcmd
set lazyredraw

set shell=pwsh

" plugins
call plug#begin()
" Plug 'SirVer/ultisnips'
Plug 'honza/vim-snippets'
"Plug 'nightsense/snow'                        " 主题
"Plug 'arcticicestudio/nord-vim'               " 主题
Plug 'projekt0n/github-nvim-theme'
"Plug 'hzchirs/vim-material'                   " 主题
Plug 'vim-airline/vim-airline'                 " taskbar美化插件
Plug 'vim-airline/vim-airline-themes'
Plug 'ctrlpvim/ctrlp.vim'                      " 模糊查找文件
call plug#end()

" keyboard
nmap j gj
nmap k gk

" theme
" snow theme
"colorscheme snow
"let g:airline_theme='snow_light'

" material theme
"colorscheme vim-material
"let g:material_style='dark'
"let g:airline_theme='material'

" nord theme
" colorscheme nord
" let g:airline_theme='nord'

" github theme
" github_dark_dimmed, github_dark_default, github_dark_colorblind, 
" github_light, github_light_high_contrast
colorscheme github_dark_tritanopia
let g:airline_theme='github_dark_tritanopia'

" set background=dark
set termguicolors

" taskbar
set t_Co=256                                    " 色域设置
set laststatus=2                                " 这个是安装字体后 必须设置此项 
let g:airline_powerline_fonts = 1   
let g:airline#extensions#tabline#enabled = 1
let g:airline_section_c = ''                    " 禁用文件名
"let g:airline_left_sep = '|'
"let g:airline_right_sep = '|'

" font, not for terminal
" set guifont=等距更纱黑体\ SC:h13
```

基于 Vim Syntax 的内置高亮：

```vim
"syntax on

" 设置高亮组背景透明 
highlight Normal guibg=NONE ctermbg=NONE
highlight NonText guibg=NONE ctermbg=NONE
highlight LineNr guibg=NONE ctermbg=NONE
highlight SignColumn guibg=NONE ctermbg=NONE
highlight EndOfBuffer guibg=NONE ctermbg=NONE
" 设置高亮关键词为粗体
" - cterm 终端效果
" - gui 独立 GUI 软件效果
" highlight Identifier cterm=bold gui=bold
highlight Type cterm=bold gui=bold
highlight Statement cterm=bold gui=bold
```

基于 Nvim Treesitter 的高亮：
* 安装命令行 `tree-sitter`
* 安装插件 `nvim-treesitter/nvim-treesitter` 
* 配置 `init.lua` 并执行 `:TSUpdate`


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
