" only for nvim on Ubuntu22.04
" 1. install nvim above 0.8.x: sudo add-apt-repository ppa:neovim-ppa/unstable && sudo apt update && sudo apt install neovim
" 2. install vim-plug: sh -c 'curl -fLo "${XDG_DATA_HOME:-$HOME/.local/share}"/nvim/site/autoload/plug.vim --create-dirs https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim'
" 3. mkdir -p ~/.config/nvim/init.vim
" 4. vim commad: `:PlugInstall`
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
"Plug 'davidhalter/jedi-vim'                    " python 自动补全
call plug#end()

" theme
syntax on
set termguicolors
colorscheme catppuccin-macchiato 
let g:airline_theme="catppuccin" "latte, frappe, macchiato, mocha
" 背景透明
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


