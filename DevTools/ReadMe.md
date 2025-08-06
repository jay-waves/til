品味如下:
- 命令行优先, GUI 不丑
- 可以复杂, 但不能繁琐丑陋, 不能绕圈子解决问题 (点名 lua, cmd). 
- 开源优先. 一般不自行编译.
- 多平台, 跨平台功能不过分阉割. 优先支持 Windows + Debian. 
- 控制资源占用. 不喜欢 Web 套壳和 Java. 
- 功能不搞大杂烩.

***

- ~~Mail Master 网易邮箱大师~~ 已换为 Thunderbird
- ~~**Zotero7**, 文献管理~~ 不常用, 一般用文件夹 + obsidian 管理.
- SumatraPDF, Adobe Acrobat DC.
- ~~GoldenDict, EuDic, 本地词典~~
- ~~Jane Reader, windows 下 EUPB 阅读器~~ 改为其分支版本 Readest
- Obsidian, 主力笔记软件
- ~~AFFiNE, 笔记软件, 非主力~~
- VSCode
- DrawIO, 绘制流程图和框架图. 我更喜欢 ppt.
- VirtualBox, WSL, WMare Player, Docker Desktop. 容器/虚拟机软件
- Datagrip, DBeaver, 数据库软件.
- Terminal, ~~Alacritty~~, 命令行终端模拟器.
- Jupyter Lab (kernel: ipython, ihaskell, wolfram14)
- GVim90, Nvim
- SimpleNotePad
- NeatConverter 电子书格式转换
- FileConverter 右键菜单中的格式转换工具
- [Files](https://github.com/files-community/Files)
- CrystallDiskInfo, 用于检查存储硬盘状态
- DiskGenius
- ~~OpenHardwareMonitor~~, 用于监控硬件, UI挺好看也挺全. LibreHardwareMonitor 支持得更好.
- **Traffic Monitor**, 任务栏网速/硬件状态监控, 小巧.
- **ExplorerPatcher**, windows选项集合(看不惯自带设置可以用, 选项密度更高).
- **ContextMenuManager4.0**, 右键管理
- FileTypesMan 文件关联管理器, 不太好用但无竞品
- **Geek**, 删除程序用
- rufus, 刷盘工具
- Display Driver Uninstaller (DDU), 显卡驱动彻底删除
- cleanmgr+ 硬盘清理工具
- autoruns, 微软家的管理启动项工具
- process explorer, 微软的进程工具
- **Luna**, 自动切换 Windows10 亮暗主题. Win11 可以用 **Auto Dark Mode**,
- G-Helper. 华硕电脑管家, 开源精简版.
- StartKiller, 删掉任务栏 windows 图标
- ~~BitDock, 给 windows 加上 dockhub~~
- **PowerToys**, 微软的小工具集
- **Snipaste** 截图与贴图
- 7zip, winRaR 压缩软件. 推荐 PeaZip.
- [Carnac](https://github.com/Code52/carnac) 键盘可视化
- [keyviz](https://github.com/mulaRahul/keyviz) 键盘可视化
- Calibre, 管理电子书库
	- ebook-convert, 书籍格式转换
	- ebook-meta, 编辑查看元信息
	- ebook-veiwer
	- calibredb
- CPU-Z 又一个硬件信息收集器
- GPU-Z 
- Little Registry Cleaner
- SyncThing 电脑文件夹同步工具
- RustDesk 远程桌面 (RDP) 软件

### Medai

- **PotPlayer**
- ffmpeg, 终端音视频编解码器
- GIMP2, 开源界的 Photoshop
- ImageMagick, 命令行的 Photoshop
- mpv, 终端视频播放器
- imagicMagick, 终端图片批处理工具
- **HandBrake**, 视频编码压缩和转换
- LosslessCut 快速裁切视频, 无转码导出
- Microsoft ClipChamp, 傻瓜式轻量剪辑, 对我够用了.
- Captura, 便携录屏软件, 依赖 ffmpeg
- XnViewMP, XnConvert, 图片库查看器

### Network

- V2ray, V2rayN
- Clash Verge, Clash for Windows
- hiddify
- shadowcocks
- UsbEAm Hosts Editor
- WinSCP, 远程 ftp
- tcpview, 微软的网络工具

### Hack & Security

- **CyberChefs, 加密/编码/压缩/数据分析工具**
- Wireshark (tShark), 抓包工具.
- imhex, 原始数据查看器
- BurpSuite, ZAP (free)
- CryptoMator, 硬盘加密工具
- VeraCrypt, 也是硬盘加密工具, 可以自己选择算法
- SET, 社工库
- sqlmap, sql 注入工具
- Sandboxie Pro 沙盒
- Chidra, 逆向工具

## Cli

- PowerShell7. Windows 下使用的命令行.
	- PSReadline 插件
	- posh-git 插件
- Clink. cmd prompt 加强工具, 已弃用 cmd.
- Bash 相关见 [/System/Shell](../System/Shell/Bash.md)

<br>

- **pandoc**
- ~~helix. 终端编辑器, 键位反人类, 已弃用.~~
- nivm, neovide, ctags
- **mpv**, 终端音视频播放器.
- httpie
- bat, cat 的增强版
- duf, dust, du 磁盘空间查看
- iconv, uchardet [字符编码](../System/Development/字符编码/字符编码.md)工具
- fq 类 yq, jq, 还支持各类二进制文件查看
- fd, find 的加强版, 使用正则表达式而不是通配符.
- rg (ripgrep)
- fzf, 模糊查找工具, 将 fd, find, rg 用管道传给它, 来进一步查找.
- tokei, 统计代码行数. cloc 替代品.
- yazi, 终端文件浏览器
- zoxide
- imagicMagick, 终端图片批处理工具
- mdv, 终端 Markdown 文件渲染器
- ~~wolframscript, 数学引擎.~~ 不常用
- ~~gopass, 密码管理器~~
- ~~age, 加密工具 (不能自定义密码套件, 封装 [PGP](../Network/ApplicationL5/PGP.md), 开箱即用)~~
- bandwhich, 查看当前占用宽带的应用和IP
- [when-changed](https://github.com/joh/when-changed)
- [repren](https://github.com/jlevy/repren)
- Graphviz + Doxygen. 代码可视化工具.
- sl, genact 摸鱼用
- neofetch, Figlet 装逼用.
- hl, lnav, 日志浏览器
- piper 文字转语音 TTS 工具
- hyperfine: `time` 的替代品, 命令行 Benchmarking.

Windows 上一些 GNU 工具移植, 见 [GNUwin32](https://gnuwin32.sourceforge.net/packages.html)
- file. 类似的还有 trid, 
- 
## Browser Plugins

- Dark Reader
- pic in pic
- gobal speed (edge)
- markdownload
- scroll to top button
- shiny picture in picture 画中画
- timetab (edge), Tabliss (firefox)
- vimium C
- Ublock Origin, WebRTC Leak Shield (edge)
- 隐形滚动条 (edge)
- TemperMonkey
	- Github 增强 - 高速下载
	- m3u8-downloader
	- CSDN Greener
	- 知乎增强, 知乎美化, 知乎下载器
	- 网盘直链下载助手
- Tab Stash (firefox)

