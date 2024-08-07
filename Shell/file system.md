`df`，`mount`，`fdisk`，`mkfs`，`lsblk`

## 文件夹管理

### `cd`

change working directory.

### `ls`, `tree`

`ls -i`: inode

### `mkdir`

Makes a new directory.  

```bash
mkdir dirname
```

You can use this to create multiple directories at once within your current directory.

```bash
mkdir 1stDirectory 2ndDirectory 3rdDirectory
```

You can also use this to create parent directories at the same time with the -p (or --parents) flag. For instance, if you wanted a directory named 'project1' in another subdirectory at '/samples/bash/projects/', you could run:

```bash
mkdir -p /samples/bash/projects/project1
mkdir --parents /samples/bash/projects/project1
```

Both commands above will do the same thing.
If any of these directories did no already exist, they would be created as well.

### `pwd`

<pre>
$ pwd
/home/JayWaves/src
</pre>

### `ln`

`ln file1 file2` 创建硬连接  

硬链接缺点: 不能跨磁盘, 不能释放空间, 不能指向文件夹.
file2 -> file1 inode -> file1 data_block, 两者使用同一inode. 删除file1不影响file2

`ln -s file1 file2`创建软连接, file2文件类型为l

file2 -> file2 inode -> file2 data_block (l) -> file1 -> file1 inode -> file1 data_block.   
删除file1会使file2失效

## 硬盘

### `df`

Shows disk usage. (disk free)

### `du`

Shows the disk usage of files or directories. For more information on this command check this [link](http://www.linfo.org/du.html)

```bash
du [option] [filename|directory]
```

Options:

- `-h` (human readable) Displays output it in kilobytes (K), megabytes (M) and gigabytes (G).
- `-s` (supress or summarize) Outputs total disk space of a directory and supresses reports for subdirectories. 

Example:

```bash
du -sh pictures
1.4M pictures
```

### `quota`

Shows what your disk quota is.  

```bash
quota -v
```

### `ldparm`

SATA/ATA 磁盘更改以及性能分析.

## 文件元信息

### `stat`

### `file`

确定文件类型

<pre>
$ file index.html
 index.html: HTML document, ASCII text
</pre>

### `chattr`

修改文件属性, 比文件权限更底层.

```bash
# 保护文件不被意外删除, 可用不可修改标记:
sudo chattr +i /my/critical/file
```

### `wc`

Tells you how many lines, words and characters there are in a file. 

```bash
$ wc demo.txt
7459   15915  398400 demo.txt
```

Where `7459` is lines, `15915` is words and `398400` is characters.

- `wc -l`, lines
- `wc -m`, characters 
- `wc -w`, words
- `wc -c`, bytes

## 文件索引

### `find` !

```bash
$ find . -name README.md
$ find . -iname REAME.md # case insensitive
$ find /home/user1 -name '*.png'
```

### `locate`

任意位置索引(需要先建立索引): `locate "pattern"`

### `zoxide`

## 文件管理

### `mv` !

Moves a file from one location to other.  

```bash
mv filename1 filename2
```

Where `filename1` is the source path to the file and `filename2` is the destination path to the file.

Also it can be used for rename a file.

```bash
mv old_name new_name
```

### `cp` !

复制文件.

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

### `repren`

[`repren`](https://github.com/jlevy/repren) 用于批量重命名文件, 或在多个文件中搜索替换内容.

```bash
      # 将文件、目录和内容全部重命名 foo -> bar:
      repren --full --preserve-case --from foo --to bar .
      # 还原所有备份文件 whatever.bak -> whatever:
      repren --renames --from '(.*)\.bak' --to '\1' *.bak
      # 用 rename 实现上述功能（若可用）:
      rename 's/\.bak$//' *.bak
```

### `rm` !

删除文件. 常用 `-r` 递归删除文件夹, `-f` 强制删除(无确认)

```bash
rm filename
```

### `touch` !

Updates access and modification time stamps of your file. If it doesn't exists, it'll be created.

```bash
touch filename
```

Example:

```bash
$ touch trick.md
```

### `truncate`

`truncate` 用于创建[稀疏文件](https://zh.wikipedia.org/wiki/稀疏文件), 即高效创建空文件. `fallocate` (用于 ext4, xfs, btrf, ocfs2), `xfs_mkfile`, `mkfile` (solaris, mac os)

### `when changed`

https://github.com/joh/when-changed, 自动监控文件改动

## 安全

managerment of files access permissions, see [system permission](system%20permission.md)

### `gopass`

密钥存储

### `cryptomator`

硬盘级加密

### `openssl, gpg, age`

文件加密工具, `age` 不支持自己配置算法, 开箱即用.

```bash
tar cvfz "files.tar.gz" "files"
age -e -p -o "files.tar.gz.age" "files.tar.gz"
```

## 文件压缩

早期压缩格式, 如 `gzip`, 仅支持单文件压缩. 压缩多文件时 (尤其是小文件较多时), 需要搭配 `tar` 归档程序将多文件归档为单个 `.tar` 文件, 再执行压缩. 

后续改进压缩算法, 自身已支持归档, 即支持多文件压缩.

### `7zip`

指定压缩比:
- `mx0` 无压缩
- `mx1` 最快速压缩
- `mx9` 最高压缩比

指定压缩算法:
- `zip` 
- `7z` 默认, 内核为 LZMA 算法, 高压缩率并支持**强加密**.
- `gzip`, `.gz` 常用于 Unix , 但只支持单个文件 (需要配合 `tar`)
- `bzip2`, `.bz2` 比 GZIP 压缩率略好, 但速度更慢.
- `xz` 基于 LZMA2 算法, 高压缩率.
- `tar` 非压缩算法, 用于模拟 TAR 归档. 注意 7zip 不会负责归档.

```sh
7z a archive.7z  -mx9 "C:\My\Files"

7z a -t7z encrypted.7z "..." -p123456 -mx=9

7z a -ttar archive.tar "..."
7z a -tgzip -mx5 archive.tar.gz archive.tar
```

> 7z 加密有个问题, 无法保密文件夹结构. 新版加入了参数 `-mhe=on`

### `tar`

`tar` 本身是归档工具, 不指定压缩算法时, 不进行压缩. `myfile.tar.gz` 的意思为, 用 `gzip` 压缩算法和 `tar` 归档程序打包的文件.

- `c` create, 创建新归档文件
- `x` extract
- `v` verbose
- `f` file, 指定文件名
- `zjJ` 指定压缩算法 gzip/bzip2/xz
- `k` keep exisitn files

```sh
tar -xvf a.tar
```