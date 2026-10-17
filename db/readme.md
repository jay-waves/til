
## 数据库系统

Database Management Systems (DBMS): 

* Data Modeling & Design
    * Normalization 
    * Relational Agebra
    * Data Dependencies
    * Tables (Schema)
* SQL (Query Languages)
    * CRUD (create, read, update, delte)
    * Key and constraints
* Query Processing 
* Storage & Indexing 
* Transactions & Concurrency & Recovery

Data Processing:
* OLTP (Online Transaction Processing):   
    user facing transactions, realtime, low latency, high-concurrency ( often CRUD small amounts of data)
* OLAP (Online Analytical Processing):   
    decision support, batch and complex analytical queries (ad hoc. full table scans and large data query)
* Vectors

## 数据密集型程序

数据密集型程序：

* databases：存储数据
* caches：缓存中间结果
* search indexes：数据索引
* stream & batch processing：异步地、批量地 处理数据流

常见数据系统架构：

<img src="../assets/db/ddis-f1-1.avif" alt="ddis-f1-1" width="500">

## Data Models 

* 关系型数据模型：SQL 
* 文档型数据模型：Json (Self-Contained, Schema-less)
* 对象型数据模型：Graph-based

SQL 和 NoSQL 的核心区别是：SQL 可以通过 [Normalization](../relational-theory/normalization.md) 来减少信息冗余，加速搜索、修改，适合多对多关系。而 NoSQL 直接存储文本，信息的局部性更高（对人类阅读更友好），但不利于索引，适合一对多关系。


### JSON 

```json
{
	"key" : "value",
	"list" : [],
	"dict" : {}
}
```

### Graph-Based 

当多对多关系复杂时，单次查询就需要多个 JOIN 语句，SQL 需要缓存大量索引来解决性能瓶颈。此时可以使用 Graph-based Data Models，更利于信息遍历和存储“关系”。[邻接表图](../../algo/graph/readme.md)，可用两个关系表表示（节点表和边表），如下：

```sql
CREATE TABLE vertices (
	vertex_id  integer PRIMARY KEY,
	label      text,
	properties jsonb
);

CREATE TABLE edges (
	edge_id     integer PRIMARY KEY,
	tail_vertex integer REFERENCES vertices (vertex_id),
	head_vertex integer REFERENCES vertices (vertex_id),
	label       text,
	properties  jsonb
);

CREATE INDEX edges_tails ON edges (tail_vertex);
CREATE INDEX edges_heads ON edges (head_vertex);
```

以 Neo4j 图数据库的 Cypher 查询语言为例：

```cypher
CREATE
(namerica  :Location {name:'North America', type:'continent'}),
(usa       :Location {name:'United States', type:'country'}),
(idaho     :Location {name:'Idaho',         type:'state'}),
(lucy      :Person {name:'Lucy'}),
(idaho) -[:WITHIN]->  (usa)   -[:WITHIN]-> (namerica),
(lucy)  -[:BORN_IN]-> (idaho)
```

```cypher
MATCH
(person) -[:BORN_IN]->  () -[:WITHIN*0..]-> (:Location {name:'United States'}),
(person) -[:LIVES_IN]-> () -[:WITHIN*0..]-> (:Location {name:'Europe'})
RETURN person.name
```

## 参考

*Designing Data-Intensive Applications*. Martin Kleppmann. 2017. 1e.

https://github.com/ByteByteGoHq/system-design-101

https://aosabook.org/en/v2/zeromq.html

https://aosabook.org/en/posa/high-performance-networking-in-chrome.html

Open-MX-IOAT.pdf

https://aosabook.org/en/posa/secrets-of-mobile-network-performance.html

https://aosabook.org/en/posa/warp.html

[The Architecture of Open Source Applications (Volume 2)
Scalable Web Architecture and Distributed Systems](https://aosabook.org/en/v2/distsys.html)
