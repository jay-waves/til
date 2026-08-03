---
revised: 2026-01-27
---

品味如下:
- 命令行优先，GUI 不丑
- 可以复杂，但不能繁琐丑陋，不能绕圈子解决问题。
- 开源优先。
- 多平台，跨平台功能不过分阉割。
- 控制资源占用。不喜欢 Java。不喜欢 Chome 套壳，可以直接运行在浏览器 
- 功能不搞大杂烩

***

## Windows GUI

* 邮箱：~~Thunderbird~~，切换为网页应用 GMail、QQMail 
* PDF 阅读：~~SumatraPDF、Readest~~，切换为 `pdf.ts`
* PDF 编辑：Stirling PDF、~~Adobe Acrobat DC~~
* EPUB 阅读：~~Readest（Jane Reader 分支）~~，切换为 `epub.ts`
* ~~正式笔记：Obsidian~~ 
* 简单笔记：NVim 
* 代码：~~VSCode~~、 Zed 
* 流程与框架图绘制：DrawIO、PPT、reveal.js
* 终端：Windows Terminal，Alacritty（Linux 平台）
* 截图与贴图：Snipaste
* 压缩：[7-Zip-zstd](https://github.com/mcmilk/7-Zip-zstd)
* ~~电子书库管理：Calbibre~~
* ~~媒体库管理：Jellyfin~~
* 远程桌面：向日葵、RustDesk 
* 应用卸载器：geek, O&O AppBuster
* 本地照片库：[Allusion](https://github.com/RafaUC/Allusion)

### 监控

* 刷盘：refus 
* 流量和硬件监控: ~~Traffic Monitor~~ 改为 [taskbar-monitor](https://github.com/leandrosa81/taskbar-monitor)
* 硬件监控：LibreHardwareMonitor 
* CPU 性能测试：CPU-Z
* 硬盘管理：CrystallDiskInfo、DiskGenius 

### 媒体

* ~~Potplayer~~
* mpv 
* ffmpeg 
* ImageMagick 
* GIMP3 
* HandBrake：视频编码与压缩
* LosslessCut：无损快速裁切视频
* ClipChamp：轻剪辑，仅支持 MP4 导出
* OBS-Studio：录屏
* OnePhoto：图片库

### Hack & Security

- **CyberChefs**
- Wireshark，抓包工具
- imhex，二进制查看
- SET，社工库
- Chidra，逆向工具
- binwalk，二进制分析工具

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

## AI 

* [OpenRouter](https://openrouter.ai/models)
* grok, chatgpt 
* piper 文字转语音 TTS 工具 （非常轻量）

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
