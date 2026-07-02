数据密集型程序：
* databases：存储数据
* caches：缓存中间结果
* search indexes：数据索引
* stream & batch processing：异步地、批量地 处理数据流

常见数据系统架构：

![ddis-f1-1|500](http://oss.jay-waves.cn/til/ddis-f1-1.avif)

## 网络应用的性能

Network Performance:
- throughput. measured by the ability of a **single endpoint** to process a large volume of messages
- response time. 
	- latency. measured by the time for messages to travel between **two endpoitns**.
	- queueing delays. 
	- jitter (抖动) variation or occasional outliers in network delay 


Metrics: 
* QPS (Query per Second) 
* Latency 
* **Mean Response Time**: useful to estimating throughput limits 
* **p50, p95, p99 (percentiles)**: useful to measure how bad the outliers (*tail latencies*) are. Even if only one backend call is slow, the chance of getting a slow result increases if an end-user request requires multiple backend calls (木桶效应)


*HOL, Head-of-Line* (护航效应). small number of slow requests to hold up the processing of subsequent requets. 

![ddis-f2-1|600](http://oss.jay-waves.cn/til/p99.avif)

### Batching 

ZeroMQ 将多个消息合并为一次请求处理, 避免频繁地系统调用, 从而增加系统的吞吐量. 但*批处理 (batching)* 会增加延迟, 同一批次的最先抵达的网络包, 其延迟也最大. 最佳实践是, 当消息速率超过网络带宽 (bps, msg/s...) 时, 打开网络批处理, 避免消息排队; 批处理应在整个网络栈的尽可能上层开启, 减少整体资源开销.

TCP 中的批处理算法被称为 *Nagle's Algorithm*, 通常网络场景总有消息排队 (queueing effect), 推荐开启.

![](../../attach/net-0mq-batch-processing.avif)

![](../../attach/net-zmq-batch-processing.avif)

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