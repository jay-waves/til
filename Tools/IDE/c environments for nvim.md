
- 包管理器: vcpkg
- 构建工具: ninja
- 构建系统: cmake
- 操作系统: Debian GNU/Linux 11
- 编辑器: nvim
- 编辑器插件管理器: [vim-plug](https://github.com/junegunn/vim-plug)
- 编辑器 PLS 前端: ycm
- 编辑器 PLS 后端: clangd

## 安装编辑器及插件管理器

安装 0.8.x 以上的 nvim, 通过第三方源:
```bash
sudo add-apt-repository ppa:neovim-ppa/unstable && sudo apt update && sudo apt install neovim
```

安装插件管理器 vim-plug
```bash
sh -c 'curl -fLo "${XDG_DATA_HOME:-$HOME/.local/share}"/nvim/site/autoload/plug.vim --create-dirs https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim'

```

配置 nvim:
```bash
mkdir -p ~/.config/nvim/init.vim
```

编写 init.vim 修改 nvim 配置, 下载 ycm

```vim
call plug#begin('~/.local/share/nvim/plugged')
Plug 'ycm-core/YouCompleteMe'
call plug#end()
```

## 配置 ycm

注意, Ycm 需要 clangd 版本大于 12. 而系统自带的版本一般为 11, 所以不要手动指定 clangd 的程序路径, 让 ycm 自行下载需要的版本即可.
`let g:ycm_clangd_binary_path='/usr/bin/clangd'` 删掉这个 nvim 配置.

YCM 需要我们手动编译, 需要确保电脑已有 `build-essential, cmake, git, clang, ninja, make` 等编译套件. 这里仅开启 ycm 对 C 系列语言和 Ninja 的支持.

```bash
cd ~/.local/share/nvim/plugged/YouCompleteMe
python3 install.py --clangd-completer --ninja
```


```vim
" in ~/.config/nvim/init.vim
let g:ycm_use_clangd=1

" 补全窗口
let g:ycm_add_preview_to_completeopt="popup"
set pumheight=15     " 限制补全窗口高度

" 预览窗口
set splitbelow       " 预览窗口出现在下方
set previewheight=15 " 限制预览窗口高度
                     " 补全结束后关闭预览窗口
autocmd CompleteDone * if pumvisible() == 0 | pclose | endif 

set omnifunc=youcompleteme#Complete

" 静态诊断窗口
let g:ycm_show_diagnostics_ui = 1
let g:ycm_show_diagnostics_ui_in_insert_mode = 0  " 插入模式不弹诊断窗口
let g:ycm_disable_for_files_larger_than_kb = 500  " 超过 500KB 的文件不启用诊断
let g:ycm_show_diagnostics_ui_in_insert_mode = 0  " 禁用键入时的实时代码诊断, 避免卡顿
autocmd InsertLeave * :YcmCompleter Diagnostic    " 推出键入模式时诊断一次
autocmd BufWritePost * :YcmCompleter Diagnostic   " 写入时诊断一次

" 跳转到函数定义
nnoremap <silent> <F12> :YcmCompleter GoToDefinition<CR>
" vim 自带很多跳转:
" ctrl-o, ctrl-i 在前后位置跳转
" gi 跳转到上次插入位置
" g; 跳转到上次修改位置
" :jumps, :jump [n]
" gD 跳转到该标识符的全局定义
```


## 配置 clangd

配置 `.clangd` 文件, 给 clangd 传一些额外参数. 如 `C_INCLUDE_PATH, CPLUS_INCLUDE_PATH`, 避免 clangd 无法自行找到标准库和第三方库的头文件位置.

```yaml
ompileFlags:
  Add: [
    -fno-cxx-modules,                               # 禁止模块补全, 只补全包含的头文件
    -isystem, /usr/include/c++/10,                  # 标准库头文件
    -isystem, /usr/include/x86_64-linux-gnu/c++/10  # 体系架构头文件, 和平台相关
    -isystem, ./include                             # 项目头文件位置
  ]
# Compiler: ...
```

## 安装 vcpkg

```bash
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg && ./bootstrap-vcpkg.sh
export PATH=/path/to/vcpkg:$PATH
```

## 配置 cmake

让 cmake 将最终编译命令输出到 compile_comands.json 文件中, 让 ycm 和 clangd 读取, 用于代码诊断和补全.
```cmake
# in CMakeLists.txt
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
```

配置 CmakePreset, 使 CMake 识别 vcpkg 安装的依赖包. 需要 CMake 版本大于 3.19

```json
// in CMake Presets.json
{
  "version": 3,
  "configurePresets": [
    {
      "name": "vcpkg",
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build",
      "cacheVariables": {
        "CMAKE_TOOLCHAIN_FILE": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake",
        "CMAKE_EXPORT_COMPILE_COMMANDS": "ON",
      }
    }
  ]
}
```

这是用户个性化的 CMakePreset, 该文件应该加入 `.gitignore`, 仅用于本地配置.
```json
// in CMakeUserPresets.json
{
    "version": 3,
    "configurePresets": [
      {
        "name": "default",
        "inherits": "vcpkg",
        "environment": {
          "VCPKG_ROOT": "path/to/vcpkg"
        }
      }
    ]
  }
```

## 预构建

生成 cmake database, 其中包含 compile_commands.json. 该文件会被输出到 `./build` 下, 可以软链接到项目根目录下.
```bash
cmake -S . -B build -G Ninja --preset default 
```