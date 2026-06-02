LFS (Linux From Scratch) 建议创建一个新硬盘分区用于存放 构建出的 Linux 系统, 未来直接从这个分区启动 Linux, 从而完全脱离宿主系统. 我这里的方式是, 在 QEMU 中运行新系统, 不完全脱离宿主系统. 

- 宿主机 (Ubuntu): 构建 LFS, 准备 `lfs.qcow2` 虚拟硬盘文件, 挂载到 `/mnt/lfs`. 在 `/mnt/lfs` 上编译完整的 LFS 系统, 并安装 grub + kernel
- QEMU: 加载 `lfs.qcow2` 从而启动 LFS 系统.

## 1. 创建硬盘分区

创建一个虚拟硬盘, 然后挂载为网络磁盘:
```bash
qemu-img create -f qcow2 lfs.qcow2 50G

# nbd(network nlock device)
sudo modprobe nbd
sudo qemu-nbd --connect=/dev/nbd0 lfs.qcow2

# 分区
sudo parted /dev/nbd0
# 在 parted 交互界面输入, 给 UEFI 分配 301MB, Linux 分配剩余部分
(parted) mklabel gpt 
(parted) mkpart primamy ext4 1MiB 301MiB
(parted) set 1 esp on
(parted) mkpart primary ext4 301MiB 100%
(parted) quit

# 格式化分区
sudo mkfs.vfat -F32 /dev/nbd0p1
sudo mkfs.ext4 /dev/nbd0p2

# 挂载分区
sudo mkdir -p /mnt/lfs
sudo mount /dev/nbd0p2 /mnt/lfs
sudo mkdir -p /mnt/lfs/boot/efi
sudo mount /dev/nbd0p1 /mnt/lfs/boot/efi

# 检查一下
lsblk /dev/nbd0
NAME     MAJ:MIN RM  SIZE RO TYPE MOUNTPOINTS
nbd0      43:0    0   50G  0 disk
├─nbd0p1  43:1    0  300M  0 part /mnt/lfs/boot/efi
└─nbd0p2  43:2    0 49.7G  0 part /mnt/lfs
```

给新系统分配 50GB 空间 (虽然最小只需要 10GB). 内部分区:
- `/boot` 大概 200MB, 存储内核和启动程序
- `/boot/efi` 用于 UEFI 加载
- `/home`
- `/usr`, LFS 文件系统中 `/bin, /lib, /sbin` 都软链接在 `/usr` 下对应文件夹. 如 `ln -sv /mnt/lfs/usr/bin /mnt/lfs/`
- `/opt` 大概 10GB, 用于安装一些大型的软件环境, 和 `/usr` 中结构隔离开.
- `/tmp` 10GB 以下

## 2. 下载必要的包

直接看原文: https://www.linuxfromscratch.org/lfs/view/stable/chapter03/introduction.html

下文不会描述一些需要技巧的细节, 详见[原文](https://www.linuxfromscratch.org/lfs/view/). 注意, 不要漏掉任何一个细节.

## 3. 构建临时工具

LFS 要构建一个最小的独立于宿主机的 Linux 系统, 系统中编译器和 C 库都是自行构建, 而不信任或使用宿主机的相关编译工具链. 这要求编译过程需要逐步自举.

在 Pass1, 构建一个交叉编译器 `x86_64-lfs-linux-gnu-gcc`, 不依赖 glibc. 用它编译安装临时的 glibc. 这个工具 (3.1) 安装在 `$LFS/tools` 下

在 Pass2, 用 Pass1 中 cross-compiler gcc 和 glibc 来构建一个完整的 GCC. 这个 gcc 也是临时的, 用来编译一众临时工具. 这些工具 (包括 gcc) 都临时安装 `$LFS` 中, 但是最终会被替换, 因为它们不能保证独立于宿主环境.

chroot 后, 我们才构建最终的 glibc 与 gcc, 安装在新系统的标准目录下.

### 3.1 构建交叉编译工具链

LFS 用 `x86-lfs-linux-gnu` 来和宿主系统平台 `x86-unkown-linux-gnu` 做区分, 这样虽然是同一平台, 但是两个编译链是隔离不能互通的. 这样编译产物可以和宿主系统充分隔离.

#### binutils 

以 binutils 为例:
- `--prefix` 安装目录
- `--with-sysroot` 指定根目录, 从这个位置开始相对寻找库
- `--target` 指定为交叉编译环境 x86-lfs-linux-gnu (仅改动了 vendor)

```bash
cd /mnt/lfs/src/binutils-2.44/build

../configure --prefix=/mnt/lfs/tools \
             --with-sysroot=/mnt/lfs \
             --target=x86_64-lfs-linux-gnu   \
			... # 详见原文

make
make install
```

#### gcc

处理 gcc 源码外, 还需要第三方 mpfr, gmp, mpc 库.

- `--with-newlib`: 此时还没有能用的 C 运行时 (glibc), 让交叉编译工具用精简的标准库 newlib 即可
- `--without-headers`: 此时还没有 glibc, 因此无需头文件
- `--disable-shared`: 此时还没有 glibc, 让 gcc 总是静态链接
- 禁用对 `threads, atomic, gomp, quadmath, libssp, vtv, libstdcxx` 的支持
- 只开启对 c/c++ 语言的编译支持

#### linux API

安装 glibc 需要的系统调用接口, 这部分在 kernel 源码中, 需要将它拷贝到 LFS 的头文件目录

```bash
cd /mtn/lfs/src/linux/
make mrproper   # 清理一些和平台/体系结构相关的头文件
make headers 
cp -rv usr/include /mnt/lfs/usr
```

#### glibc & libstdc++

使用已经安装在 `/mnt/lfs/tools` 中的交叉编译器来编译 glibc, 配置其支持的平台为 5.4 及更新的 Linux Kernel, 并且使用 `/mnt/lfs/usr/include` 中的头文件.
```bash
cd build
../confgiure      \
		--prefix=/usr \
		--host=$LFS_TGT \
		--enable-kernel=5.4 \
		--with-headers=/mnt/lfs/usr/include \
		--disbale-nscd \
		libc_cv_slibdir=/usr/lib
```

### 3.2 构建交叉编译用的临时工具

用 3.1 中编译出的交叉编译器, 编译一些临时工具, 这些工具被链接到 3.1 中安装的 glibc 动态库. 由于我们没有 `chroot`,  所以事实上这些临时工具还不能使用.

```
m4 ncurses bash corutils(+hostname) diffutils file findutils gawk grep gzip 
make patch sed tar xz binutils gcc
```

这里的 binutils, gcc 在 3.1 中用宿主机环境编译过一次, 放在 `$LFS/tools` 中, 其交叉编译的 Triplets 是 x86_64-lfs-linux-gnu. 这里编译第二次 (记得把原本 `./build` 删除), 用 3.1 中编译出的交叉编译器编译, 安装在我们的新系统根目录 `$LFS` 下.

由于编译这个 gcc 过程中, 用到的 gawk, make 等仍是宿主机的软件, 因此不能说是完成了自举, 这仍是一个临时的 gcc. (比如宿主机工具的版本等无法控制, 软件可能有内嵌的宿主机绝对路径)

### 3.3 Chroot

```bash
chroot "$LFS" /usr/bin/env -i \
		HOME=/root            \
		TERM="$TERM"          \
		PS1='(lfs chroot) \u:\w\$' \
		PATH=/usr/bin:/usr/sbin    \
		/bin/bash --login
```

LFS 中需要遵循 [FHS 标准](https://refspecs.linuxfoundation.org/FHS_3.0/fhs/index.html)创建文件目录结构, 然后创建一些文件. 用户和属组配置通常是为了兼容 Unix 传统, 实际上访问受限硬件和系统服务有更通用的做法. 修改 /root 之后, 可以将 `$LFS/tools` 删除.

```bash
cat > /etc/hosts << EOF
127.0.0.1 localhost $(hostname)
::1       localhost
EOF

cat > /etc/passwd << "EOF"
root:x:0:0:root:/root:/bin/bash 
bin:x:1:1:bin:/dev/null:/usr/bin/false 
daemon:x:6:6:Daemon User:/dev/null:/usr/bin/false 
messagebus:x:18:18:D-Bus Message Daemon User:/run/dbus:/usr/bin/false uuidd:x:80:80:UUID Generation Daemon User:/dev/null:/usr/bin/false nobody:x:65534:65534:Unprivileged User:/dev/null:/usr/bin/false
EOF

cat > /etc/group << "EOF"
root:x:0: 
tty:x:5: 
disk:x:8: 
lp:x:9: 
dialout:x:10: 
audio:x:11: 
video:x:12: 
utmp:x:13: 
cdrom:x:15: 
adm:x:16: 
messagebus:x:18: 
input:x:24: 
mail:x:34: 
kvm:x:61: 
uuidd:x:80: 
wheel:x:97: 
users:x:999: 
nogroup:x:65534:
EOF
```

## 构建完整 LFS 系统

