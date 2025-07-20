### vsc workspace

vscode 每个工作区代表一个项目, 每个项目可以开启不同的插件. 也就是, 不同项目支持不同的语言环境. 使用插件 Project Manager 可以管理工作区.

目前我配置的工作区有:
- C++/Python: VS22 + Python3.10
- Java: openjdk-22
- JupyterNotebook: Wolfram Engine14 + Python3.10
- Latex: texlive
- Go: go-1.21.0
- Remote C++/Python: remote Debian in WSL

工作时, 将项目目录软链接到工作区目录下, 使用 `mklink` 命令.

## plugins list

AI:
- Copilot
- AWS Toolkit + Amazon Q
- IntelliCode

Latex:
- Latex WorkShop
- Ultra Math Preview
- Latex Utilities

Wolfram Mathematica:
- Wolfram Language
- Wolfram Language Notebook

Python:
- Python Indenting
- Python 
- Python Debugger
- Pylance
- Mypy
- Jupyter xxx

C:
- GDB Debug
- Clangd
- Clang-Format
- C/C++ Extension Pack
- LLVM (IR)
- CMake + CMake Tools

Other:
- Regex Previewer
- Markdown Preview
- Makrdown All in One
- Dev Contianers + WSL + REmote SSH
- **Project Manager**
- **Rewrap**
- Command Variable
- Gitignore + GitGraph
- **Indenticator**
- SQLite Viewer
- **Vim**
- Bookmarks
- Better Comments
- Snippets Manager

Themes:
- Noctis
- Plaenight Theme
- **Github Theme**
- Catppuccin + Icon
- Material Themes + Icon
- Nord
- Catppuccin Noctis