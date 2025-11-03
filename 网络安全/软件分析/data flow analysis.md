## 数据流分析

### 流敏感

- sensitive to control flow 
- intraprocedural analysis 

一个程序语句 `a = b + c` 对数据流的影响?
1. **使用**了变量 b, c
2. 使变量 a 的原定义值**失效**
3. **新定义**了变量 a 的值

#### Reaching Definitions Analysis 

Transfer functions:
`Out[b] = f(In[b]) = Gen[b] + (In[b] - Kill[b])`
- `Gen[b]` 基本块产生的 definitions
- `In[b]` 从 前序基本块 传入的 definitions
- `Kill[b]` 基本块对变量的重定义, 意味着原 definition 失效.

从控制流入口开始, 正向分析, 研究 "definition" 的有效范围

```
input: CFG = (N, E, Entry, Exit) 

// 初始化
	out[Entry] = {}
	
	For all nodes i
		out[i] = {}
		
	ChangedNodes = N
	
// 工作集迭代
	while ChangedNodes not empty {
		Remove i from ChangedNodes
		in[u] += out[p], for all predecessors p of i
		oldout = out[i]
		out[i] = f(in[i], i) // f = gen[i] + (in[i] - kill[i])
		if (oldout != out[i]) {
			for all successors s of i
				add s to ChangedNodes
		}
	}
```

#### Variables Liveness Analysis 

在某个程序位置 p, var x is live, 意味着从 p 开始的控制流中 x 会被使用. 反向分析, 从变量 x 的使用位置开始, 反向找到它的定义.

`in[b] = Use[b] + (out[b] - Def[b])`
- `Use[b]` 基本块 b 使用的变量 
- `out[b]` 从 后继基本块 传播 (propagate) 来的 var liveness
- `Def[b]` 基本块 b 定义的变量

```
input: CFG = (N, E, Entry, Exit)

// 边界条件
in[Exit] = {}

// 初始化
For each b.b. B other than Exit 
	in[B] = {}
	
// 迭代
While (changes to any in[] occur) {
	For each basic block B other than Exit {
		out[B] += (in[s]), for all successors s of B
		in[B] = f(out[B], B) // f = Use[B] + (out[B] - Def[B])
	}
}
```

什么情况无法收敛? 

### 路径敏感分析

数据流 + 条件可达性分析:
- SAT (命题逻辑公式可满足性判定): 仅限布尔类型的谓词逻辑判定
- SMT (Satisfiability Modulo Theories): 混合类型的谓词逻辑判定, 包括 未解释函数 / 布尔表达式 / 线性算术 / 字符串表达式 / 非线性算术等. 

流行的 SMT Solver 有: CVC4, [Z3](https://github.com/Z3Prover/z3). 