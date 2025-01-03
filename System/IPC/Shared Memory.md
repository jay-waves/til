---
revised: 2024-04-09
copyright:
  - Dave Marshall (1899)
  - Jay Waves (2024)
---

## SystemV API

函数原型在 `<sys/shm.h>` 或 `linux/uapi/linux/shm.h`. SystemV 是比较老的接口, Linux 目前仅支持兼容.

### 获取共享内存段

```c
// Create a shared memory segment, returns segment id (`shmid`) once
// successes. Also used to get shmig of an existing shared segment.
// key: an access value associated with semaphore id.
// size: size in bytes of requested shm
// shmflg: initial access permissions, 
int shmget(key_t key, size_t size, int shmflg);

{ // example:
	if ((shmid = shmget(key, size, shmflg)) == -1) {
		perror("shmget: shmget failed");
		exit(1);
	} else {
		frpint(stderr, "shmget: shmget returned %d\n", shmid);
		exit(0);
	}
}
```

### 控制共享内存段

```c
// prototype
int shmctl(int shmid, int cmd, struct shmid_ds *buf);
// cmd is one of the following:
// - SHM_LOCK, superuser lock this shm in memory.
// - SHM_UNLOCK, superuser unlock this shm.
// - IPC_STAT, with read permission, get status info written in &buf 
// - IPC_SET, owner, creator or superuser set effective user and group identification and access permissions.
// - IPC_RMID, remove shm 
// buf: shmid_ds is defined in sys/shm.h

// data structure for shared memory metadata
struct shmid_ds {
    struct ipc_perm shm_perm;     /* Ownership and permissions */
    size_t          shm_segsz;    /* Size of segment (bytes) */
    time_t          shm_atime;    /* Last attach time */
    time_t          shm_dtime;    /* Last detach time */
    time_t          shm_ctime;    /* Last change time */
    pid_t           shm_cpid;     /* PID of creator */
    pid_t           shm_lpid;     /* PID of last shmat(2)/shmdt(2) */
    shmatt_t        shm_nattch;   /* No. of current attaches */
    // Other necessary fields
};

// example:
if ((rtrn = shmctl(shmid, cmd, shmid_ds)) == -1_ {
	perror("shmctl: shmctl failed");
	exit(1);
	}
```

### 链接/脱离共享内存段

```c
// prototype
// return pointer of `shmaddr` at the head of shm assocaiated with 
// a valid `shmid`
void* shmat(int shmid, const void *shmaddr, int shmflg);
int shmdt(const void *shmaddr);

// example
#include <sys/types.h> 
#include <sys/ipc.h> 
#include <sys/shm.h> 

static struct state { /* Internal record of attached segments. */ 
			int shmid;     // shmid of attached segment
			char *shmaddr; // attach point
			int shmflg;    // flags used on attach
		 } ap[MAXnap];     // State of current attached segments.
int nap; /* Number of currently attached segments. */

...

char *addr;               // address work variable
register int i;           // work area
register struct state *p; // ptr to current state entry
...

p = &ap[nap++];
p->shmid = ...
p->shmaddr = ...
p->shmflg = ...

p->shmaddr = shmat(p->shmid, p->shmaddr, p->shmflg);
if(p->shmaddr == (char *)-1) {
		 perror("shmop: shmat failed");
		 nap--;
		} else
		(void) fprintf(stderr, "shmop: shmat returned %#8.8x\n",
p->shmaddr);

... 
i = shmdt(addr);
if(i == -1) {
		perror("shmop: shmdt failed");
		} else {
	(void) fprintf(stderr, "shmop: shmdt returned %d\n", i);

for (p = ap, i = nap; i--; p++)   
	if (p->shmaddr == addr) *p = ap[--nap];
	
}
```

### Example

process `shm_server` and `shm_client` communicate with each other.

```c
// shm_server.c, create string and shared memory portion
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>

#define SHMSZ     27

main(){
	char c;
	int shmid;
	key_t key;
	char *shm, *s;

	// name our shared memory segment with '5678'
	key = 5678;

	//Create the segment
	if ((shmid = shmget(key, SHMSZ, IPC_CREAT | 0666)) < 0) {
		perror("shmget");
		exit(1);
	}

	// attach the segment to our data space
	if ((shm = shmat(shmid, NULL, 0)) == (char *) -1) {
		perror("shmat");
		exit(1);
	}

	// pu sth into shm
	s = shm;
	for (c = 'a'; c <= 'z'; c++)
		*s++ = c;
	*s = NULL;

	// wait until the other process changes the first character of 
	// our memory to '*' indicating that it has read what we put
	// there.
	while (*shm != '*')
		sleep(1);

	exit(0);
}
```

```c
// shm_client.c, attach itself to the created shared memory portion, 
// and print the string from shm_server
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>

#define SHMSZ 27

main(){
	int shmid;
	key_t key;
	char *shm, *s;

	// shm key
	key = 5678;

	// locate the shm
	if ((shmid = shmget(key, SHMSZ, 0666)) < 0) {
		perror("shmget failed");
		exit(1);
	}

	// Now we attach the segment to our data space.
	if ((shm = shmat(shmid, NULL, 0)) == (char *) -1) {
		perror("shmat failed");
		exit(1);
	}

	// read what server put
	for (s = shm; *s != NULL; s++)
			putchar(*s);
	putchar('\n');

	// finally, change the first character of segment to '*', 
	// indicating that we've read the shm.
	*shm = '*';

	exit(0);
}
```

## POSIX API

POSIX 共享内存实际是内存映射 (mapped memory) 的变种.

| POSIX shared memory | mapped memory |
| ------------------- | ------------- |
| `shm_open()`, with fewer options        | `open()`      |
| `shm_unlink()`      | `close()`, but not delete     |

```c
// prototypes
int shm_open(const char *name, int oflag, mode_t mode); // Create or open a shared memory object
int shm_unlink(const char *name); // Remove a shared memory object's name
int ftruncate(int fd, off_t length); // Change the size of an open file

// Data structure for shared memory header
struct shm_header {
    int data_ready;
    pthread_mutex_t mutex;
    pthread_cond_t cond_var;
    // Other necessary fields
};


```

## Mapped Memory

虚拟内存比共享内存更高效和简洁, 同时虚拟内存允许应用在逻辑上占用更大的内存空间.

```c
#include <sys/types.h>
#include <sys/mman.h>
/*
 establish a mapping of a named file system object into address space of a 
 process with mmap()
 1. fd = open(...)
 2. mmap(..., PROT_READ | PROT_WRITE, MAP_SHARED | MAP_PRIVATE, fd, ...)
 3. munmap() // unmap pages of memory
 4. close(fd)
*/
void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset); 
int munmap(void *addr, size_t length);
```


- `mlock(), munlock(), mlockall()`
- `msync(), fsync()`
- `sysconf(), mprotect(), brk(), sbrk()`

## More Examples

see [shmget](../../src/shmget.c), [shmop](../../src/shmop.c), [shmctl](../../src/shmctl.c)

1. use `shmget.c` to initialized semaphore
2. use `shmctl.c` to control shared memory
3. use `shmop.c` to access shared memory