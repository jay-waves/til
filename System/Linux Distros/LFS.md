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

下文不会描述一些需要技巧的细节, 详见[原文](https://www.linuxfromscratch.org/lfs/view/)

## 3. 构建

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

#### glibc 



### 3.2 构建工具



### 3.3 

