" windows 下放在 ~/AppData/Local/nvim/init.vim

set number                                      " 行号
set tabstop=4                                   " tab 宽度
set shiftwidth=4                                " 缩进宽度
set expandtab                                   " 将tab解释为空格
set cursorline                                  " 高亮当前行
syntax on
set smartindent  
set showcmd
set wrap                                        " 自动换行
set ignorecase                                  " 搜索时忽略大小写
set smartcase                                   " 如果搜索包含大写字符，则自动启用大小写敏感

" font, not for terminal
set guifont=Source\ Code\ Pro:h13

" plugins
call plug#begin()
Plug 'SirVer/ultisnips'                         " Snippets 片段补全插件
Plug 'honza/vim-snippets'
Plug 'nightsense/snow'
Plug 'arcticicestudio/nord-vim'
Plug 'hzchirs/vim-material'
"Plug 'vim-syntastic/syntastic'                 " 语法检查插件
Plug 'vim-airline/vim-airline'                  " taskbar美化插件
Plug 'vim-airline/vim-airline-themes'
Plug 'ctrlpvim/ctrlp.vim'                       " 模糊查找文件
"Plug 'davidhalter/jedi-vim'                    " python 自动补全
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
colorscheme nord
let g:airline_theme='nord'

" github theme
" github_dark_dimmed, github_dark_default, github_dark_colorblind, 
" github_light, github_light_high_contrast
colorscheme github_dark_tritanopia
let g:airline_theme='github_dark_tritanopia'

" - Statemnet 控制流关键词, if, for 等
" - Error 显示错误文本
" - PreProc 预处理关键词, define 等
" - Type 数据类型关键词
" - Special 特殊字符, 转义字符
" - Underlined 下划线文本, 如链接
" - LineNr 行号
" - CursorLine 当前行高亮提示
" - CusorColumn 当前列高亮提示
" - Visual 视觉模式选择
" - SignColumn 行号与文本之间的列, 如显示断点
" - StatusLine 状态栏
" - EndOfBuffer 文件末尾空行符号
" - NoneText 非文本字符, 如空行符号和行尾符号
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

set background=dark
set termguicolors

" taskbar
set t_Co=256                                    "色域设置
set laststatus=2                                "这个是安装字体后 必须设置此项 
let g:airline_powerline_fonts = 1   
let g:airline#extensions#tabline#enabled = 1
let g:airline_section_c = ''                    " 禁用文件名
"let g:airline_left_sep = '|'
"let g:airline_right_sep = '|'

" activate syntastic
let g:syntastic_always_populate_loc_list = 1
let g:syntastic_auto_loc_list = 1
let g:syntastic_check_on_open = 1
let g:syntastic_check_on_wq = 0
let g:syntastic_python_checkers = ['mypy']

" 配置 ultisnips 代码补全
" pip install neovim
let g:UltiSnipsExpandTrigger="<tab>"        " TAB 展开
let g:UltiSnipsJumpForwardTrigger="<c-j>"   
let g:UltiSnipsJumpBackwardTrigger="<c-k>"

