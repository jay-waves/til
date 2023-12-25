"vimrc在windows用户目录urs/yjw中
"设置固定字体
"set guifont=Microsoft_YaHei_Mono:h13,Cascadia_Mono_ExtraLight:h13
"set guifont=Consolas:h13
set guifont=JetBrainsMonoNL_NFM:h13,Microsoft_YaHei_Mono:h13

"plug in, 默认目录在usr/yjw/vimfiles/plugged.  
":PlugUpdate :PlugInstall, PlugClean
call plug#begin()
Plug 'nightsense/snow'
"Plug 'hzchirs/vim-material'
"Plug 'scrooloose/nerdtree', {'on': 'NERDTreeToggle'} 
Plug 'vim-syntastic/syntastic' " 语法检查插件
"Plug 'Yggdroot/LeaderF', { 'do': './install.sh' } 	
Plug 'vim-airline/vim-airline' " taskbar美化插件
"Plug 'enricobacis/vim-airline-clock'  taskbar美化插件
Plug 'vim-airline/vim-airline-themes'
Plug 'ctrlpvim/ctrlp.vim'  " 寻找文件, 模糊查找
Plug 'davidhalter/jedi-vim' " python 自动补全
"Plug 'vim-scripts/taglist.vim'
"Plug 'itchyny/lightline.vim'
call plug#end()

set nu 
set nowrap
set mouse=a
" 平滑滚动, 默认一次滚5行, 太快了
map <ScrollWhellUp> <C-Y>
map <ScrollWheelDown> <C-E>
set lines=37   
set columns=80 

" 映射键盘
" 按视觉行上下移动, 配合wrap用
nmap j gj
nmap k gk
nmap <C-b> :NERDTree<CR>
"统计中文字数, 原理是匹配中文字符, 然后替换为自身,
"nmap <F2> :%s/[\u4E00-\u9FCC]/&/g<CR> 
"选中后, 添加括号
"vmap <F2> s()<ESC>P 

"自动缩进
set smartindent
"autoindent, cindent
set shiftwidth=4 
set softtabstop=4
set tabstop=4
"将tab改变为空格
set expandtab

"字符编码
let &termencoding=&encoding
set fileencoding=utf-8

"snow配色配置汇总, 还需要开启Plug
colorscheme snow
set background=light
"set termguicolors
let g:airline_theme='snow_light'

"material配色汇总
"colorscheme vim-material
"set background=light
"set background=dark
"let g:material_style='palenight'
"let g:airline_theme='material'
set termguicolors

"配色
"colorscheme snow
"背景暗黑还是明亮
"set background=light

"taskbar
"let g:airline_theme='snow_light'
set t_Co=256 "色域设置
set laststatus=2
"这个是安装字体后 必须设置此项 
let g:airline_powerline_fonts = 1   
let g:airline#extensions#tabline#enabled = 1
let g:airline_section_c = ''  " 禁用文件名
"let g:airline_left_sep = '|'
"let g:airline_right_sep = '|'
            
"语法高亮
syntax enable

"syntastic
set statusline+=%#warningmsg#  
set statusline+=%{SyntasticStatuslineFlag()}  
set statusline+=%*  

let g:syntastic_python_checkers=['python']
let g:syntastic_error_symbol='×'
let g:syntastic_warning_symbol='!'
"let g:syntastic_always_populate_loc_list = 1 总是更新错误,否则需要:Errors来更新错误
let g:syntastic_auto_loc_list = 1  
let g:syntastic_check_on_open = 0 
let g:syntastic_check_on_wq=1

"窗口设定
set guioptions-=m "去掉菜单
set guioptions-=T "去掉工具栏
set guioptions-=r 
set guioptions-=l "大写则在分割窗口时隐藏
set guioptions-=b

"auto code complete
"autocmd FileType pyhon set omnifunc=pythoncomplete#Complete
filetype plugin indent on
set completeopt=menu
set wildmenu  "自动补全命令时候使用菜单式匹配列表  
set omnifunc=syntaxcomplete#Complete
autocmd FileType python set omnifunc=python3complete#Complete

"用tab补全, 和补全切换
 function! CleverTab()
     if strpart( getline('.') ,0,col('.')-1) =~'^\s*$'
         return "\<Tab>"
     else    
         return "\<C-N>"
     endif   
 endfunction
 inoremap <Tab> <C-R>=CleverTab()<CR>

"""python
"set pythonthreedll=python38.dll

""" F5 to run sh/python3
"map <F5> :call CompileRunGcc()<CR>`
"func! CompileRunGcc()
"    exec "w"
"   if &filetype == 'python'
"        exec "!python %"
"  endif
"endfunc
