Splay Tree 是一种自调整的[二叉搜索树](binary-tree.md)。不严格维护平衡条件，而是进行一种 *Splay* 操作：**每次访问一个节点，就将其旋转到根**。

Splay Tree 单次操作不稳定，最坏有 $O(n)$ 。但均摊复杂度是 $O(\log n)$ 。适合单线程，局部性强的场景，比链表优雅一些。

## splay 

#### zig 

```ascii
Before:

      P
     /
    X

Access: X

After:

      X
       \
        P
```


#### zig-zig

```
Before:

        G
       /
      P
     /
    X

Access: X

After:

        X
         \
          P
           \
            G
```

#### zig-zag 

先对 P 左旋，再对 G 右旋

```
Before:

        G
       /
      P
       \
        X

Access: X

After:

        X
       / \
      P   G
```