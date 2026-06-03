`cd`, `ls`, `tree`, `mkdir`, `pwd`, `ln`, 
`rm`, `cp`, `mv`, 
`touch`, `truncate`, 
`find`, `locate`, `zoxide`
`openssl`, `gpg`, `age`
`tar`, `7zip`, `gzip`


## 文件管理

用户使用 Shell 时, 当前所在目录被称为工作目录.

```bash
cd /home/yjw/temp               # 改变工作目录
pwd                             # 展示工作目录的路径
ls -ia                          # 展示工作目录
```

### 创建文件

#### `touch` 

更新文件 AccessTime 和 ModificationTime 元信息, 如果文件不存在, 则创建文件.

Updates access and modification time stamps of your file. If it doesn't exists, it'll be created.

```bash
touch filename
```

#### `truncate`

`truncate` 用于创建[稀疏文件](https://zh.wikipedia.org/wiki/稀疏文件), 即高效创建空文件. `fallocate` (用于 ext4, xfs, btrf, ocfs2), `xfs_mkfile`, `mkfile` (solaris, mac os)

### 创建文件夹

```bash
mkdir dir1 dir2                                    # 创建新文件夹

# 指定完整路径, 如果不存在文件夹, 会递归创建
mkdir -p /samples/bash/projects/project1      
mkdir --parents /samples/bash/projects/project1
```

### 删除, 移动, 复制文件夹及文件

删除文件. 常用 `-r` 递归删除文件夹, `-f` 强制删除(无确认)

```bash
rm filename
rm -r \xxx\somedir     # 递归删除文件夹
rm -f \xxx\somfile     # 强制删除, 无确认
```

`mv` 移动文件位置, 也可用于重命名文件.

```bash
mv path_to_file1 path_to_file2 
```

`cp` 复制文件.

```bash
cp src_file_path dest_file_path
```

复制文件时, 获取进度:
```bash
pv
pycp           # see https://github.com/dmerejkowsky/pycp
progress       # https://github.com/Xfennec/progress
rsync --progress

# 仅限 block 块拷贝
dd status=progress
```

### 创建文件夹链接

`ln file1 file2` 创建硬连接. 软硬链接区别详见 [vfs](linux-vfs.md)

`ln -s file1 file2` 创建软连接, 文件类型为 `l` 

## 文件查询

### `find`

`find` 找到含有字符串'score'的文件名, 并列出其路径: `find /path/to/dir -type f | grep score`

使用find命令会自动查询文件夹*及其子文件夹*的内容, 比如`find /dir -name file_name`

找到含有字符串'score'的文件名, 并列出其路径: `find /path/to/dir -type f | grep score`

最常用的命令是 `find`

```bash
$ find . -name README.md
$ find . -iname REAME.md # case insensitive
$ find /home/user1 -name '*.avif'
```

### `locate`

locate 可以在全系统范围查询, 但需要提前建立索引. 类似 windows everything.

### `zoxide`

类似 `cd`, 并且可以学习访问历史. 通过模糊匹配, 来实现路径自动跳转.

## 文件归档

早期压缩格式, 如 `gzip`, 仅支持单文件压缩. 压缩多文件时 (尤其是小文件较多时), 需要搭配 `tar` 归档程序将多文件归档为单个 `.tar` 文件, 再执行压缩. 

后续改进压缩算法, 自身已支持多文件和文件夹压缩, 也支持加密[^1]. 因此, 归档概念仅在 Linux 下比较常见.

[^1]: 显然不太符合 Unix 的 "Do one thing, and do it well" 风格, 高手还是喜欢 tar + xz + gpg ...

```bash
tar -xvf a.tar              # 不指定压缩算法时, 不压缩
tar -xvfz a.tar.gz          # gzip + tar
```

`tar` 常用指令:
- `c` create, 创建新归档文件
- `x` extract
- `v` verbose
- `f` file, 指定文件名
- `zjJ` 指定压缩算法 gzip/bzip2/xz
- `k` keep exisitn files

## 文件压缩

常见有界面压缩工具有 7zip, winrar, bandzip, peazip 等, 推荐 7zip, 跨平台且开源.

### 常见压缩算法

| 算法             | 压缩率        | 压缩/解压速度 | 运存占用 | 场景             | 描述 (同名算法有变种)                           |
| ---------------- | ------------- | ------------- | -------- | ---------------- | -------------------------------- |
| DEFLATE          | 中            | 快            | 低       | 打包, 兼容性环境 | LZ77 + Huffman 熵编码 |
| LZMA             | 极高          | 慢            | 高       | 软件分发, 归档   | LZ77 + 范围 熵编码                |
| Zstandard (zstd) | 高            | 快            | 中       | 实时传输         | LZ77 + ANS 熵编码              |
| BWT              | 高 (文本最优) | 慢            | 高       | 文本压缩         |  BWT + MTF + 熵编码                                |
| LZ4              | 低            | 极快        | 低         |  小文件, 文本文件               | LZ77, 无熵编码                                |

*字典编码 (Dictionary Coding)*, 指构建一个滑动窗口, 将数据中重复序列换位更短的字典引用 (偏移 + 长度). 字典编码是对字符串的无损压缩, 消除冗余重复, 速度依赖于字典和滑动窗口大小. 常用 LZ77 算法. 

*熵编码 (Entropy Coding)* 是利用数据的统计特性, 为高频符号分配更短的编码, 从而逼近理论最小熵. 编码后, 变为不定长比特流. Huffman 算法一般以字节为单位划分符号, 算术编码 / 范围编码 (LZMA 使用) / ANS (Zstd 使用) 则以更小区间 (低至比特) 为单位, 压缩率可接近香农极限, 但计算复杂度越高.

### 7zip

指定压缩比:
- `mx0` 无压缩
- `mx1` 最快速压缩
- `mx9` 最高压缩比

指定压缩格式:
- `tar` 非压缩算法, 用于模拟 TAR 归档. 注意 7zip 命令行工具不会负责归档.
- `zip`, zip 和 gzip 格式都默认使用 Deflate 算法. 
- `gzip`, `.gz` 常用于 Unix , 但只支持单个文件 (需要配合 `tar`). 
- `7z` 默认, 基于 LZMA 算法. 7z 格式支持**强加密**.
- `bzip2`, 基于 BWT 算法.
- `xz` 基于 LZMA2 算法.
- `lz4` 
- `zstd` 

```sh
7z a archive.7z  -mx9 "C:\My\Files"

7z a -t7z encrypted.7z "..." -p123456 -mx=9

7z a -ttar archive.tar "..."
7z a -tgzip -mx5 archive.tar.gz archive.tar
```

> 7z 加密有个问题, 无法保密文件夹结构. 新版加入了参数 `-mhe=on`

## 文件加密

```bash
tar cvfz "files.tar.gz" "files"
age -e -p -o "files.tar.gz.age" "files.tar.gz" # 用后缀名标识算法和工具: tar + gz + age
```