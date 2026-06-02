## memcpy 

```c
void *memcpy(void *dest, const void *src, size_t n);
```

主要涉及的难点：
* 处理好页对齐
* 辅助编译器优化：循环展开，适时用 SIMD 宽指令优化

这里用 [facebook/folly](https://github.com/facebook/folly/blob/main/folly/memcpy.S) 的实现为例：
* 小块内存避免循环（减少分支数量）
	* 先拷贝前 `k` 个字节，再拷贝后 `k` 个字节，允许中间重复拷贝，换取缓存行的效率。比如先拷贝 `[0..7]` 再拷贝 `[5..12]` 完成 13 字节拷贝。否则，写判断对齐的逻辑，需要很多分支，对于小内存不值得。
	* 如果要考虑 `dst`, `src` 区域有重叠，需要先将数据全部 `load` 到寄存器，再考虑 `store`，否则可能覆盖还未读取的数据。
* 大块内存（指 256B 以上），用 AVX 指令优化，并考虑页对齐

```c
#include <immintrin.h> // AVX Intrinsics v2

static inline void copy_8_16(unsigned char *d, const unsigned char *s, size_t n) {
    uint64_t a = *(const uint64_t *)(s);
    uint64_t b = *(const uint64_t *)(s + n - 8);

    *(uint64_t *)(d) = a;
    *(uint64_t *)(d + n - 8) = b;
}

static inline void copy_17_32(unsigned char *d, const unsigned char *s, size_t n) {
    __m128i a = _mm_loadu_si128((const __m128i *)s);
    __m128i b = _mm_loadu_si128((const __m128i *)(s + n - 16));

    _mm_storeu_si128((__m128i *)d, a);
    _mm_storeu_si128((__m128i *)(d + n - 16), b);
}

static inline void copy_33_64(unsigned char *d, const unsigned char *s, size_t n) {
    __m256i a = _mm256_loadu_si256((const __m256i *)s);
    __m256i b = _mm256_loadu_si256((const __m256i *)(s + n - 32));

    _mm256_storeu_si256((__m256i *)d, a);
    _mm256_storeu_si256((__m256i *)(d + n - 32), b);
}

static inline void copy_65_128(unsigned char *d, const unsigned char *s, size_t n) {
    __m256i a0 = _mm256_loadu_si256((const __m256i *)(s));
    __m256i a1 = _mm256_loadu_si256((const __m256i *)(s + 32));

    __m256i b1 = _mm256_loadu_si256((const __m256i *)(s + n - 64));
    __m256i b0 = _mm256_loadu_si256((const __m256i *)(s + n - 32));

    _mm256_storeu_si256((__m256i *)(d), a0);
    _mm256_storeu_si256((__m256i *)(d + 32), a1);

    _mm256_storeu_si256((__m256i *)(d + n - 64), b1);
    _mm256_storeu_si256((__m256i *)(d + n - 32), b0);
}

static inline void copy_0_128(unsigned char *d, const unsigned char *s, size_t n) {
    if (n == 0) return;

    if (n <= 8) {
        if (n >= 4) {
            *(uint32_t *)d = *(const uint32_t *)s;
            *(uint32_t *)(d + n - 4) = *(const uint32_t *)(s + n - 4);
        } else if (n >= 2) {
            *(uint16_t *)d = *(const uint16_t *)s;
            *(uint16_t *)(d + n - 2) = *(const uint16_t *)(s + n - 2);
        } else {
            *d = *s;
        }
        /*
	        对于 x86, 2B/4B/8B 的非对齐 load/store 也可以执行。
	        * 但是对于 RISC 指令级，非对齐的指令访问，可能导致 Page Fault 异常陷入
	        * std c 也要求 *(uint32_t *)s 要求 s 满足对齐，否则是未定义行为
        */
    } else if (n <= 16) {
        copy_8_16(s, d, n);
    } else if (n <= 32) {
	    copy_17_32(s, d, n);
    } else if (n <= 64) {
	    copy_33_64(s, d, n);
    } else {
	    copy_65_128(s, d, n);
    }

```

提供一个纯 C 版本实现（无 SIMD），可以感受一下上述代码的意图：

```c
static inline void copy_17_32(uint8_t *d, const uint8_t *s, size_t n) {
	memcpy(d, s, 16);
	memcpy(d + n - 16, s + n - 16, 16);
}
```

对于大数据块，先处理地址对齐，再拷贝对齐后的数据。注意 `loadu` 是不要求对齐的指令，`store` 采用了需要对齐的版本。

```c

static inline size_t align_forward_32(unsigned char **d,
                                      const unsigned char **s,
                                      size_t n) {
    uintptr_t misalign = (uintptr_t)(*d) & 31;

    if (misalign == 0) {
        return n;
    }

    size_t prefix = 32 - misalign;

    if (prefix > n) {
        prefix = n;
    }

    copy_0_128(*d, *s, prefix);

    *d += prefix;
    *s += prefix;

    return n - prefix;
}

static inline void copy_loop_128_aligned_dst(unsigned char *d,
                                             const unsigned char *s,
                                             size_t n) {
    size_t i = 0;

    /*
     * d must be 32-byte aligned here.
     */
    while (i + 128 <= n) {
        __m256i a0 = _mm256_loadu_si256((const __m256i *)(s + i + 0));
        __m256i a1 = _mm256_loadu_si256((const __m256i *)(s + i + 32));
        __m256i a2 = _mm256_loadu_si256((const __m256i *)(s + i + 64));
        __m256i a3 = _mm256_loadu_si256((const __m256i *)(s + i + 96));

        _mm256_store_si256((__m256i *)(d + i + 0),  a0);
        _mm256_store_si256((__m256i *)(d + i + 32), a1);
        _mm256_store_si256((__m256i *)(d + i + 64), a2);
        _mm256_store_si256((__m256i *)(d + i + 96), a3);

        i += 128;
    }

    copy_0_128(d + i, s + i, n - i);
}

void *my_memcpy_fast(void *dst, const void *src, size_t n) {
    unsigned char *d = dst;
    const unsigned char *s = src;

    /*
     * No overlap handling.
     * This is memcpy-style only, not memmove.
     */
    if (n == 0 || d == s) {
        return dst;
    }
    
    if (n <= 128) {
	    copy_0_128(d, s, n);
	    return dst;
    }
    
    /*
     * Large-block path:
     * copy a prefix so destination becomes 32-byte aligned.
     */
    n = align_forward_32(&d, &s, n);
    
    /*
     * Now d is 32-byte aligned unless n became 0.
     * Main loop can use aligned stores.
     */
    copy_loop_128_aligned_dst(d, s, n);

	// handle tail 
	copy_0_128(d + i, s + i, n - i);
    return dst;
}
```

## memmove

当 `src, dst` 的目标区域重叠时，`memcpy()` 行为是未定义的。此时应使用 `memmove`

## memset 