基数树是[字典树](trie.md)的变形, 数据更加紧凑.

https://en.wikipedia.org/wiki/Radix_tree

In [computer science](https://en.wikipedia.org/wiki/Computer_science "Computer science"), a **radix tree** (also **radix trie** or **compact prefix tree** or **compressed trie**) is a [data structure](https://en.wikipedia.org/wiki/Data_structure "Data structure") that represents a [space-optimized](https://en.wikipedia.org/wiki/Memory_Optimization "Memory Optimization") [trie](https://en.wikipedia.org/wiki/Trie "Trie") (prefix tree) in which each node that is the only child is merged with its parent. The result is that the number of children of every internal node is at most the [radix](https://en.wikipedia.org/wiki/Radix "Radix") r of the radix tree, where r = 2x for some integer x ≥ 1. Unlike regular trees, edges can be labeled with sequences of elements as well as single elements. This makes radix trees much more efficient for small sets (especially if the strings are long) and for sets of strings that share long prefixes.

Unlike regular trees (where whole keys are compared _en masse_ from their beginning up to the point of inequality), the key at each node is compared chunk-of-bits by chunk-of-bits, where the quantity of bits in that chunk at that node is the radix r of the radix trie. When r is 2, the radix trie is binary (i.e., compare that node's 1-bit portion of the key), which minimizes sparseness at the expense of maximizing trie depth—i.e., maximizing up to conflation of nondiverging bit-strings in the key. When r ≥ 4 is a power of 2, then the radix trie is an r-ary trie, which lessens the depth of the radix trie at the expense of potential sparseness.

As an optimization, edge labels can be stored in constant size by using two pointers to a string (for the first and last elements).

Note that although the examples in this article show strings as sequences of characters, the type of the string elements can be chosen arbitrarily; for example, as a bit or byte of the string representation when using [multibyte character](https://en.wikipedia.org/wiki/Multibyte_character "Multibyte character") encodings or [Unicode](https://en.wikipedia.org/wiki/Unicode "Unicode").


```c

#define RADIX_TREE_BITS 4  // 每个节点存储的位数
#define RADIX_TREE_MAX_HEIGHT (32 / RADIX_TREE_BITS)

struct radix_tree_node {
    void *slots[1 << RADIX_TREE_BITS];
};

struct radix_tree_root {
    struct radix_tree_node *rnode;
    int height;
};

static void radix_tree_init(struct radix_tree_root *root)
{
    root->rnode = NULL;
    root->height = 0;
}

static struct radix_tree_node *radix_tree_node_alloc(void)
{
    struct radix_tree_node *node;
    node = malloc(sizeof(struct radix_tree_node));
    if (!node) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(EXIT_FAILURE);
    }
    memset(node->slots, 0, sizeof(node->slots));
    return node;
}

static int radix_tree_insert(struct radix_tree_root *root, unsigned long index, void *item)
{
    struct radix_tree_node **nodep = &root->rnode;
    struct radix_tree_node *node = root->rnode;
    int height = root->height;
    int shift = (height - 1) * RADIX_TREE_BITS;
    int offset;

    // 如果树为空，创建根节点
    if (!node) {
        node = radix_tree_node_alloc();
        root->rnode = node;
        root->height = 1;
        height = 1;
        shift = 0;
    }

    // 查找插入点
    while (height > 0) {
        if (!*nodep) {
            *nodep = radix_tree_node_alloc();
        }
        node = *nodep;
        offset = (index >> shift) & ((1 << RADIX_TREE_BITS) - 1);
        nodep = (struct radix_tree_node **)&node->slots[offset];
        shift -= RADIX_TREE_BITS;
        height--;
    }

    offset = index & ((1 << RADIX_TREE_BITS) - 1);
    node->slots[offset] = item;

    return 0;
}

static void *radix_tree_lookup(struct radix_tree_root *root, unsigned long index)
{
    struct radix_tree_node *node = root->rnode;
    int height = root->height;
    int shift = (height - 1) * RADIX_TREE_BITS;
    int offset;

    if (!node)
        return NULL;

    // 查找路径
    while (height > 0) {
        offset = (index >> shift) & ((1 << RADIX_TREE_BITS) - 1);
        node = (struct radix_tree_node *)node->slots[offset];
        if (!node)
            return NULL;
        shift -= RADIX_TREE_BITS;
        height--;
    }

    offset = index & ((1 << RADIX_TREE_BITS) - 1);
    return node->slots[offset];
}

static void *radix_tree_delete(struct radix_tree_root *root, unsigned long index)
{
    struct radix_tree_node **nodep = &root->rnode;
    struct radix_tree_node *node = root->rnode;
    int height = root->height;
    int shift = (height - 1) * RADIX_TREE_BITS;
    int offset;
    void *item;

    if (!node)
        return NULL;

    // 查找路径
    while (height > 0) {
        offset = (index >> shift) & ((1 << RADIX_TREE_BITS) - 1);
        node = (struct radix_tree_node *)node->slots[offset];
        if (!node)
            return NULL;
        nodep = (struct radix_tree_node **)&node->slots[offset];
        shift -= RADIX_TREE_BITS;
        height--;
    }

    offset = index & ((1 << RADIX_TREE_BITS) - 1);
    item = node->slots[offset];
    node->slots[offset] = NULL;

    return item;
}

static void radix_tree_print(struct radix_tree_node *node, int height, unsigned long index)
{
    if (!node)
        return;

    if (height == 0) {
        printf("Index %lu: %p\n", index, node->slots[0]);
    } else {
        for (int i = 0; i < (1 << RADIX_TREE_BITS); i++) {
            if (node->slots[i]) {
                radix_tree_print((struct radix_tree_node *)node->slots[i], height - 1, (index << RADIX_TREE_BITS) | i);
            }
        }
    }
}


```