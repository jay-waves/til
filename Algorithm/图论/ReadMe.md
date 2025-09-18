## 图

- 是否有向?
- 是否有加权?
- 稀疏/稠密?

### 相邻

### 度数

### 路径

### 子图

### 联通

## 图数据结构

### 邻接表

Adjacency List. 每个顶点维护 `vector<pair>`, 适合稀疏图. 

```cpp
// i ---(w)--> j
vector<pair<j, w>>
```

### 邻接矩阵

Adjacency Matrix: `m[V][V]`, 适合稠密图 (边接近 $V^{2}$). `m[i][j]` 表示从节点 `i -> j` 的边, 无权重则存储 `INF`.

## 最小生成树

最小生成树 (Minimum Spanning Tree, MST) 是指在**连通加权无向图** $G=(V,E)$ 中, 找到一个树 (无环连通子图), 包含图中所有节点, 并且边的权重之和最小. 

### Kruskal 算法

```cpp 
class Graph {
private:
	int V; // 节点数
	vector<tuple<int, int, int>> edges; // (w, u, v)
public:
	Graph(int v) : V(v) {}
	
	void addEdge(int u, int v, int w) {
		edges.push_back({u, v, w});
	}
	
	int MST();
};

```

使用并查集 (Union-Find) 来判断两个节点是否处于同一连通分量, 并检测加入边是否会形成环. 均摊复杂度为 $O(E\log E)$.

```cpp
class UnionFind {
private:
	vector<int> parent, rank;
public:
	UnionFind(int n) {
		parent.resize(n);
		rank.resize(n, 0);
		for (int i = 0; i < n; ++i) parent[i] = i;
	}
	int find(int x) {
		if (parent[x] != x) 
			parent[x] = find(parent[x]);
		return parent[x];
	}
	bool unite(int x, int y) {
		int px = find(x), py = find(y);
		if (px == py) return false; 
		if (rank[px] < rank[py]) swap(px, py);
		parent[py] = px;
		if (rank[px] == rank[py]) rank[px]++;
		return true;
	}
};
```

```cpp
int Graph::MST() {
	sort(edges.begin(), edges.end()); 
	
	UnionFind uf(V);
	int mstWeight = 0;
	vector<tuple<int, int, int>> mstEdges;
	
	for (auto [w, u, v] : edges) {
		if (uf.unite(u, v)) { // 无环
			mstWeight += w;
			msgEdges.push_back({u, v, w});
		}
	}
	
	return mstWeight;
}
```

### Prim 算法

从一个节点开始, 每次选择与当前生成树连接的权重最小的边. 使用优先队列选择最小边时, 复杂度为 $O(E\log V)$.

```cpp
class Graph {
private:
	int V;
	vector<vector<pair<int, int>>> adjList; // (u, w)
public:
	void addEdge(int u, int v, int w) {
		adjList[u].push_back({v, w});
		adjList[v].push_back({u, w});
	}
	
	int MST();
};
```

```cpp
int Graph::MST() {
	vector<int> key(V, INT_MAX); 
	vector<bool> inMST(V, false); // 是否在生成树中
	vector<int> parent(V, -1); 
	// 最小堆
	priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq; 
	
	key[0] = 0; 
	pq.push({0, 0}); 
	
	while (!pq.empty()) {
		int U = pq.top().second; 
		pq.pop();
		
		if (inMST[u]) continue; 
		inMST[u] = true; 
		
		for (auto [v, w] : adjList[u]) {
			if (!inMST[v] && w < key[v]) {
				key[v] = w;
				parent[v] = u;
				pq.push({w, v});
			}
		}
	}
}
```

## 深度优先搜索

## 广度优先搜索

## 有向无环图
DAG 

## 哈密顿图

## 图着色问题
