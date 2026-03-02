## 回溯（Backtraking）

### 组合问题

从 1...n 中选取无序的 k 个数

```cpp
vector<vector<int>> res;
vector<int> path;

void dfs(int n, int k, int start) {
    if (path.size() == k) {
        res.push_back(path);
        return;
    }

    for (int i = start; i <= n; i++) {
        path.push_back(i);       // 选择
        dfs(n, k, i + 1);        // 递归
        path.pop_back();         // 撤销
    }
}
```

### 子集问题

给定数组，返回所有子集

```cpp 
vector<vector<int>> res;
vector<int> path;

void dfs(vector<int>& nums, int start) {
    res.push_back(path);   // 每个节点都是解

    for (int i = start; i < nums.size(); i++) {
        path.push_back(nums[i]);
        dfs(nums, i + 1);
        path.pop_back();
    }
}
```

### 排列问题

全排列 n 个数。注意，排列是有序的，用于组合问题会重复。

```cpp
vector<vector<int>> res;
vector<int> path;
vector<bool> used;

void dfs(vector<int>& nums) {
    if (path.size() == nums.size()) {
        res.push_back(path);
        return;
    }

    for (int i = 0; i < nums.size(); i++) {
        if (used[i]) continue;

        used[i] = true;
        path.push_back(nums[i]);

        dfs(nums);

        path.pop_back();
        used[i] = false;
    }
}
```

### N-Queens 

在 $N\times N$ 棋盘上放置 N 个皇后，使任意两个皇后不在同一行、同一列、同一对角线的情况。

转化为三个约束条件：
* 行列冲突
* 主对角线冲突，row-col 相同
* 副对角线冲突，row+col 相同

```cpp 
vector<vector<string>> res;
vector<string> board;
vector<bool> col, diag1, diag2;

void dfs(int row, int n) {
    if (row == n) {
        res.push_back(board);
        return;
    }

    for (int c = 0; c < n; c++) {
        if (col[c] || diag1[row - c + n] || diag2[row + c])
            continue;

        col[c] = diag1[row - c + n] = diag2[row + c] = true;
        board[row][c] = 'Q';

        dfs(row + 1, n);

        board[row][c] = '.';
        col[c] = diag1[row - c + n] = diag2[row + c] = false;
    }
}

vector<vector<string>> solve_n_queens(int n) {
    board = vector<string>(n, string(n, '.'));
    col = vector<bool>(n, false);
    diag1 = vector<bool>(2*n, false);
    diag2 = vector<bool>(2*n, false);

    dfs(0, n);
    return res;
}
```

### 分支限界

**分支限界 (Branch and Bound) **: 类似回溯, 但在构建解的过程中使用一个界限来剪枝, 即提前终止不可能构建出最优解的路径. 

## 分治

### 定义

分治 (英语: Divide and Conquer) , 字面上的解释是「分而治之」, 就是把一个复杂的问题分成两个或更多的相同或相似的子问题, 直到最后子问题可以简单的直接求解, 原问题的解即子问题的解的合并. 

### 过程

分治算法的核心思想就是「分而治之」. 

大概的流程可以分为三步: 分解 -> 解决 -> 合并. 

1.  分解原问题为结构相同的子问题. 
2.  分解到某个容易求解的边界之后, 进行递归求解. 
3.  将子问题的解合并成原问题的解. 

分治法能解决的问题一般有如下特征: 

-   该问题的规模缩小到一定的程度就可以容易地解决. 
-   该问题可以分解为若干个规模较小的相同问题, 即该问题具有最优子结构性质, 利用该问题分解出的子问题的解可以合并为该问题的解. 
-   该问题所分解出的各个子问题是相互独立的, 即子问题之间不包含公共的子问题. 

## 动态规划

### 背包问题

背包问题，是指在资源受限的条件下，选择一组物品使目标最优。背包问题（Knapsack Problem）通常是 NP-Complete 问题。常见目标为：
* 价值最大
* 能否凑出某个值

$n$ 个物品，第 $i$ 个物品的重量和价值分别为 $w[i],v[i]$ ，背包容量 $C$。
* 如果每个物品仅可选择一次，称为 *01 背包*
* 如果每个物品可以无限次选择，称为*完全背包*

### 

## 贪心

贪心算法 (英语: greedy algorithm) , 是用计算机来模拟一个「贪心」的人做出决策的过程. 这个人十分贪婪, 每一步行动总是按某种指标选取最优的操作. 而且他目光短浅, 总是只看眼前, 并不考虑以后可能造成的影响. 

可想而知, 并不是所有的时候贪心法都能获得最优解, 所以一般使用贪心法的时候, 都要确保自己能证明其正确性. 

### 解释

#### 适用范围

贪心算法在有最优子结构的问题中尤为有效. 最优子结构的意思是问题能够分解成子问题来解决, 子问题的最优解能递推到最终问题的最优解. [^ref1]

#### 证明

贪心算法有两种证明方法: 反证法和归纳法. 一般情况下, 一道题只会用到其中的一种方法来证明. 

1.  反证法: 如果交换方案中任意两个元素/相邻的两个元素后, 答案不会变得更好, 那么可以推定目前的解已经是最优解了. 
2.  归纳法: 先算得出边界情况 (例如 $n = 1$) 的最优解 $F_1$, 然后再证明: 对于每个 $n$, $F_{n+1}$ 都可以由 $F_{n}$ 推导出结果. 