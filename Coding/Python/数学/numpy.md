
https://numpy.org/learn/

NumPy (Numerical Python) 用于处理大规模数组和矩阵运算, 底层封装了 C 语言实现, 速度快.

## 核心概念

**数组 (Array) **: `ndarray`, 即同类型元素的 N 维数组.

**向量化操作**: 对数组整体操作, 而不是逐个循环运算. 

**广播机制**: 对于不同形状的数组, NumPy 会自动扩展, 使其能相互运算.

## 基础

### 创建数组

从列表创建:

  ```python
  import numpy as np
  arr = np.array([1, 2, 3, 4, 5])
  ```

直接创建数组:

  ```python
	zeros = np.zeros((3, 4))  # 全零数组
	ones = np.ones((2, 3))  # 全一数组
	rand_arr = np.random.rand(2, 3) # 随机数组
	arr = np.arange(0, 10, 2) # 指定步长2的数组
  
	arr.shape
	arr.dtype
	arr[1:4]
  ```

### 操作数组

矢量运算:
  ```python
	a = np.array([1, 2, 3, 4])
	a.reshape(3, 1)
	b = np.array([4, 5, 6])
	a + b  # 向量加 [5 7 9]
	a * b  # 向量乘
	2 * a  # 标量乘
  ```

矩阵运算:

  ```python
	a = np.array([[1, 2], [3, 4]])
	b = np.array([[5, 6], [7, 8]])
	print(np.dot(a, b))  # 矩阵乘法，输出: [[19 22] [43 50]]
  ```

### 统计函数

  ```python
	arr = np.array([1, 2, 3, 4, 5])
	np.sum(arr)
	np.mean(arr)
	np.std(arr)
	np.max(arr)  # 输出: 5
	np.min(arr)  # 输出: 1
  ```

## scipy

封装了 numpy 的更高层数学库