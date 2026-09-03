---
revised: 2026-01-27
---

品味如下:
- 命令行优先，GUI 扁平现代。
- 开源优先。
- 最好支持多平台。
- 控制资源占用，不喜欢大杂烩应用。
- 复杂，但不能繁琐丑陋，不能绕圈子解决问题。
- 喜欢 WEB 应用，但不喜欢 Chome 套壳的独立应用。

***

## Windows GUI

* 邮箱： GMail、QQMail WEB 、~~Thunderbird~~
* PDF 阅读：pdf.ts、~~SumatraPDF、Readest~~ 
* PDF 编辑：Stirling PDF、~~Adobe Acrobat DC~~
* EPUB 阅读：epub.ts、 ~~Readest（Jane Reader 分支）~~
* 笔记：NVim、~~Obsidian~~ 
* 代码：Zed 、~~VSCode~~
* 流程图：DrawIO、PPT、reveal.js、Mermaid.js 
* 终端：Windows Terminal，Alacritty（Linux 平台）
* 截图与贴图：Snipaste
* 压缩：[7-Zip-zstd](https://github.com/mcmilk/7-Zip-zstd)
* 远程桌面：向日葵、RustDesk 
* 应用卸载器：geek, O&O AppBuster
* 视频播放：mpv
* 视频处理：ffmpeg、HandBrake、ClipChamp 
* 图像处理：ImageMagic、GIMP3 
* 录屏：OBS-Studio （简单场景，有 WEB 应用替代品）
* 媒体库：[Allusion](https://github.com/RafaUC/Allusion)、Calibre、Jellyfin 
* 抓包：wireshark 
* 二进制：imhex 
* 手机传文件：LocalSend 

### Windows OS

* 刷盘：refus 
* 流量和硬件监控: ~~Traffic Monitor~~ 改为 [taskbar-monitor](https://github.com/leandrosa81/taskbar-monitor)
* 硬件监控：LibreHardwareMonitor 
* CPU 性能测试：CPU-Z
* 硬盘管理：CrystallDiskInfo、DiskGenius 

## Cli

- Bash 相关见 [/os/bash ](../os/bash.md)
- Powershell 相关间 [/os/powershell](../os/powershell.md)

<br>

- **[`pandoc`](http://pandoc.org/)** 
- iconv, uchardet [char-encoding](../hw/char-encoding.md)工具
- **fzf**, 模糊查找工具
- tldr
- rclone 云存储工具
- strings 读取二进制中的字符串片段
- psmux, tmux 终端多路复用器

<br>


~~[yazi](https://github.com/sxyazi/yazi) 命令行文件管理器~~ 最近更喜欢 [lf](https://github.com/gokcehan/lf)，配置如下：
* `VISUAL = 'nvim'`
* `EDITOR = 'nvim'`
* `PAGER = 'bat --pager=builtin'`
* `SHELL = 'pwsh'`


| GNU  | Rewrite-in-Rust | Powershell | Description |
| ----- | ------------- | ---------- | --------- |
| lsdisk  | duf         |    | 磁盘统计            |
| du    | dust          |    |  目录下文件体积统计（直方图） |
| grep  | ripgrep       | findstr   |     |
| find  | fd            |      |     |
| cat   | bat           | Get-Content |  |
| cloc  | tokei         |    |    |
| file  |               |    |    |
| cd    | zoxide        |    |    |
| man   | tldr          |    | 百科全书，简短版   |
| diff  | delta      |    |    |
| curl  |  xh           | Invoke-WebRequest   |    |


> Windows 上还有一些绿色（不依赖 MinGW） GNU 工具移植：[GNUwin32](https://gnuwin32.sourceforge.net/packages.html),
> `Microsoft.CoreUtils`


## Browser Plugins

- BewlyCat、BewlyBewly 
- Google Scholar Button 
- Markdownload、Obsidian Web Extracter 
- TimeTab（edge 浏览器）
- uBlock Origin、WebRTC Leak Shield 
- vimium C
- TemperMonkey
	- Github 增强 - 高速下载
	- m3u8-downloader
	- CSDN Greener
	- 知乎增强, 知乎美化, 知乎下载器
	- 网盘直链下载助手
- SuperSimple Highlighter 
- Picture in Picture
- Obsidian Web Clipper 
- Read Aloud 

## Linux GUI

also see [.bashrc](bashrc.md), here is the GUI choice:

* flameshot 截图贴图工具
* Alacritty 命令行
* chrome 比 firefox 稳定很多
* ibus with rime engine

其他命令行配置见发行版：
* [wsl](../os/vm/wsl-config.md)
* [distros-arch](../os/linux/distros-arch.md)
* [distros-fedora](../os/linux/distros-fedora.md)

## DEV

缓存目录放在默认位置即可，重点是其他位置配置地更加紧凑。

* Rust: 放到 WSL 中
* Python：放到 WSL 中
* LLVM： 放到 WSL 中
* JS： dir, cache-dir, store-dir 
* C：MSBuild Tools DIR , VCPKG_ROOT 
* GO: GOBIN, GOROOT, GOPATH 

## 各类配色

- [snow](https://github.com/haystackandroid/snow)
- [typewritter](https://github.com/logico/typewriter)
- [material](https://github.com/hzchirs/vim-material)
- nord
- github
- Catppuccin
- [solarized](https://github.com/altercation/vim-colors-solarized)
- vim-material
- Gruvbox
- Dracula 
