linux 虚拟文件系统 (VFS, virtual file system) 是接口层, 将 [linux-fs](linux-fs.md)具体实现与操作系统服务剥离开. VFS 表示为统一的**树形**目录结构, 新文件系统通过**挂载 (mount)** 的方式装载到 VFS 的 `/mnt` 目录中. 

VFS 将不同 I/O 对象的接口统一, 都通过 `struct file` 与其内部的 `struct file_operations` 虚函数表表达. VFS 负责将路径解析为对应的内部对象.
VFS 支持的文件类型包括:
- file (f), directory (d), symlink (l) --> fs 
- executable (x)
- empty (e)
- socket (s)
- pipe (p) 
- char-dev (c): 字符设备, 一次性读取
- block-dev (b): 块设备, 支持随机存取

## inode

VFS 的处理对象也是 `inode`，和默认 [ext4](ext4.md) 文件系统有同构性。下图是对象结构关系。

```ascii
User process
┌────────────────────────────────────────────────────────────┐
│ fd = 3                                                     │
│                                                            │
│ File descriptor table                                      │
│ ┌────┬──────────────────────────────┐                      │
│ │ fd │ pointer                      │                      │
│ ├────┼──────────────────────────────┤                      │
│ │ 0  │ stdin                        │                      │
│ │ 1  │ stdout                       │                      │
│ │ 2  │ stderr                       │                      │
│ │ 3  │ ─────────────────────────┐   │                      │
│ └────┴──────────────────────────┼───┘                      │
└─────────────────────────────────┼──────────────────────────┘
                                  │
                                  ▼
Kernel VFS layer
┌────────────────────────────────────────────────────────────┐
│ Open file object                                           │
│ ┌──────────────────────────────────────────────┐           │
│ │ file offset                                  │           │
│ │ open flags                                   │           │
│ │ pointer to VFS inode ───────────────────┐    │           │
│ └─────────────────────────────────────────┼────┘           │
│                                           │                │
│                                           ▼                │
│ VFS inode                                                       
│ ┌──────────────────────────────────────────────┐           │
│ │ mode / owner / size / timestamps             │           │
│ │ inode operations                             │           │
│ │ file operations                              │           │
│ │ pointer to filesystem-specific inode ───┐    │           │
│ └─────────────────────────────────────────┼────┘           │
└───────────────────────────────────────────┼────────────────┘
                                            │
                                            ▼
Concrete filesystem, e.g. ext4 / XFS / Btrfs
┌────────────────────────────────────────────────────────────┐
│ Filesystem-specific inode                                  │
│ ┌──────────────────────────────────────────────┐           │
│ │ ext4_inode / xfs_inode / btrfs_inode         │           │
│ │ filesystem-specific metadata                 │           │
│ │ block mapping / extents / trees              │           │
│ │ pointers to physical data blocks ───────┐    │           │
│ └─────────────────────────────────────────┼────┘           │
│                                           │                │
│                                           ▼                │
│ Data blocks / extents                                     │
│ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐         │
│ │ block 17     │ │ block 23     │ │ block 88     │         │
│ │ file data... │ │ file data... │ │ file data... │         │
│ └──────────────┘ └──────────────┘ └──────────────┘         │
└────────────────────────────────────────────────────────────┘
```

下图是递归查询文件 inode 的路径。

```ascii
filename: "/home/jay/a.txt"
        │
        ▼
VFS path lookup
┌────────────────────────────────────────────────────────────┐
│ dentry cache                                               │
│                                                            │
│ "/"       ─────► VFS inode for root directory              │
│ "home"    ─────► VFS inode for /home                       │
│ "jay"     ─────► VFS inode for /home/jay                   │
│ "a.txt"   ─────► VFS inode for /home/jay/a.txt             │
└────────────────────────────────────────────────────────────┘
        │
        ▼
Concrete filesystem lookup
┌────────────────────────────────────────────────────────────┐
│ Directory data on disk                                     │
│                                                            │
│ directory entry: "a.txt" -> inode number 900               │
│ inode table / inode tree: inode number 900 -> real inode   │
└────────────────────────────────────────────────────────────┘
```

### 硬链接

**硬链接**, Hard Link, 指多个目录项指向同一个索引节点, 该索引节点的被引用数增加. 该链接方法无法跨文件系统, 当所有硬链接和源文件被删除时, 源文件才会被彻底删除, 即无法立即释放空间. 

硬链接也无法指向文件夹, 因为硬链接直接指向 `inode`, 和原文件的结构是等价的, 所以文件系统无法区分哪一个才是该目录文件真正的位置, 甚至导致目录死循环.

```
dentry  name        inode
1       myfile      101
2       hardlink1   101
3       hardlink2   101
```

```
file2 -> file1 inode -> file1 data_block
```

### 软链接

**软链接**, Soft Link, 创建一个新文件, 文件内容是被链接文件的路径. 访问该软链接时, 实际通过该路径访问到另一个文件. 软连接可以跨文件系统, 目标文件被删除时, 软链接失效.

```
dentry name       inode
1      myfile     101
2      softlink   103

inode blocks 
101   ...
103   /path/to/myfile
```

```
file2 -> file2 inode -> file2 data_block (l) -> file1 -> file1 inode -> file1 data_block
```

## 实际的对象定义

```c
struct inode {
		umode_t i_mode; // permission + filetype 
		
		uid_t i_uid;  // owner user id
		gid_t i_gid;  // owner group id 
		
		struct timespec i_atime;
		struct timespec i_mtime;
		struct timespec i_ctime;
		
		loff_t i_size; // file size in bytes 
		blkcnt_t i_blocks; // num of allocated disk blocks 
		
		unsigned long i_ino; // inode numbder 
		unsigned ing  i_nlink; // hard link count 
		
		struct super_block *i_sb;  // --> fs
		struct address_space *i_mapping; // page chache mapping 
		
		const struct inode_operations *i_ops; // link, rename, new, delete 
		const struct file_operations *i_fop; // write or read file 
		
		struct list_head i_sb_list; // inode list in superblock 
		struct hlist_node i_hash; // inode hash table 
		
		spinlock_t i_lock;
		unsigned long i_state; // dirty, freeing, syncing...
		
		void *i_private; // fs-specified private data 
};
```

文件的元信息的详解请见 [os/fs/file-metadata](file-metadata.md)

文件操作的详解请见 [os/fs/linux-file-operations](linux-file-ops.md)

inode 本身有三种状态：
* inode 存在于内存中，未关联任何文件，也不处于活动使用状态
* inode 存在于内存中，正在被进程使用，通常代表一个文件。此时 `i_nlink > 0 && i_count > 0` ，并且其数据（文件元数据和文件内容）均未被改动
* inode 处于活动状态，但数据内容已经被改变，和存储介质中不同步

同一个 `super_block` 下的所有 `inode` 挂载在一个链表 `inode->i_sb_list` 中，同时为了加快查找速度，还建了一个哈希表 `i_hash` ，用于通过 `i_ino` 编号直接获取对象指针。

#### 文件描述符

文件描述符数量有上限 `RLIMIT_NOFILE` 限制. 系统能分配的最大文件描述符数由  `sizeof(int)` 限制, 记录在 `/pro/sys/fs/file-max`; 单个进程的最大文件描述符数记录在 `/proc/sys/fs/nr_open`.

`Too many open files` 错误有两种:
- `EMFILE` per-process limit : 可能是超上限, 也可能是资源泄漏 (没有关闭)
- `ENFILE` system-wide limit 

使用 `lsof -p <pid>` 查看进程的文件描述符, 用 `ulimit` 提升进程 FD 限制. 当然, 更推荐的方法, 是使用 IO 复用.

```cpp
class FD {
	int fd_;

public:
	explicit FD(int fd = -1) noexcept : fd_(fd) {}
	
	// 关闭拷贝语义
	// 允许移动语义
	
	~FD() { reset(); }
	void reset(int newfd = -1) noexcept {
		if (fd_ != -1)
			::close(fd_);
		fd_ = newfd;
	}
	int fd() const noexcept { return fd_; }
	explicit operator bool() const noexcept { return fd_ != -1; }
}
```
