## PLS

Language Server Protocol (LSP). 源于 VSCode, 旨在将 IDE 的前后端分离, 前端提供编辑功能, 后端提供代码分析和编译功能. 前后端通过 LSP 协议通信, 可以实现远程连接部署, 可以减小 IDE 体积, 按需配置. 常见 PLS 如 `gopls, pypls, clangd`

常见 IDE 功能:
- Code Completion, 代码补全.
- Inlay Hints, 代码行内提示. 
- Tool Tips, Hover Information, 悬浮提示.
- Inline Annotations, 下划线警告信息.
- Diagnostics, 代码分析建议, 定期检查代码中 "错误, 质量, 风格问题".
- Navigation, Symbol Definition Lookup, 建立符号大纲, 支持定义跳转.

独立提供代码分析建议的工具称为 Lint, 如 `golint, flake8, mypy, eslint`, 比 PLS 更轻量和独立.
