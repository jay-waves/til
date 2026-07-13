#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 13],
  source: [Understanding Deep Learning],
  revised: [2026-07-13],
  tags: ("deep-learning", "udl"),
)

#let argmin = math.op("arg min", limits: true)
#let argmax = math.op("arg max", limits: true)
#let ReLU = math.op("ReLU")
#let softmax = math.op("softmax")
#let sigmoid = math.op("sigmoid")
#let KL = math.op("KL")
#let E = math.op("E")
#let Pr = math.op("Pr")
#let Var = math.op("Var")
#let Cov = math.op("Cov")
#let ELBO = math.op("ELBO")
#let diag = math.op("diag")
#let sign = math.op("sign")

= PDF 14: Graph Neural Networks

== Motivation

Convolutional networks specialize in regular grids, and transformers specialize in sequences or complete token interactions. Many real datasets are irregular relational structures: road networks, social networks, molecules, scene graphs, meshes, or citation networks. These are graphs.

== Graph Representation

A graph has nodes $cal(V)$ and edges $cal(E)$. Nodes and edges can also have attributes. Graphs can be directed or undirected, weighted or unweighted, homogeneous or heterogeneous.

An adjacency matrix records graph structure:

$ A_(i j) = cases(1 & (i, j) in cal(E), 0 & "otherwise") $

For undirected graphs, $A$ is symmetric. Directed graphs can have asymmetric adjacency matrices. Weighted graphs use real values instead of binary entries.

Node indexing is arbitrary. If node order is permuted, the adjacency matrix changes but the graph is the same. A graph neural network must respect this:

- node-level outputs should be permutation equivariant
- graph-level outputs should be permutation invariant

== Tasks and Losses

Common graph tasks include:

- graph classification or regression: one output for the whole graph
- node classification or regression: one output per node
- edge prediction: classify or score edges or candidate edges
- graph generation: predict graph structure and attributes

Graph-level tasks use a readout that aggregates node embeddings into a graph embedding. Node tasks preserve node order equivariance. Edge tasks combine pairs of node or edge embeddings.

== Message Passing

Most GNN layers update node embeddings by aggregating information from neighbors:

$ m_i = sum_(j in cal(N)(i)) g(h_i, h_j, e_(i j)) $

$ h'_i = f(h_i, m_i) $

The aggregation must be independent of neighbor ordering. Sum, mean, max, and attention are common choices.

After $K$ message-passing layers, node $i$ can depend on nodes up to $K$ hops away. This is the graph analogue of a receptive field.

== Graph Convolution

A simple graph convolution aggregates neighboring hidden vectors using shared parameters. A common normalized form is:

$ H' = sigma(tilde(D)^(-1/2) tilde(A) tilde(D)^(-1/2) H W) $

where $tilde(A) = A + I$ adds self-loops and $tilde(D)$ is the degree matrix.

Self-loops let each node preserve and transform its own features. Normalization prevents high-degree nodes from dominating merely because they have more neighbors.

The same parameters are shared across all nodes, similar to the way CNNs share kernels across image positions.

== Graph Classification

For graph classification, each graph has its own adjacency matrix and node feature matrix. A stack of graph layers creates node embeddings. A permutation-invariant readout such as sum, mean, max, or attention pooling creates a graph-level embedding, which is fed to an ordinary classifier or regressor.

Batching graph-level tasks is straightforward when each example is a separate graph: combine graphs into a larger block-diagonal graph or process them as a batch with bookkeeping.

== Inductive and Transductive Settings

Graph-level tasks are typically inductive: training and test graphs are separate.

Node classification can be transductive: training and test nodes belong to the same graph, but only some labels are known during training. It can also be inductive when the model must classify nodes in new graphs.

The distinction matters because transductive training can use the whole graph structure, whereas inductive models must learn reusable local update rules.

== Batching Large Graphs

For very large graphs, full-batch training may be impossible. Two strategies are common:

- neighborhood sampling: sample a limited number of neighbors at each hop
- graph partitioning: train on subgraphs created by clustering or partitioning

Both approximate the full graph context to make training feasible.

== Layer Variants

Graph convolutional layers can be modified in several ways:

- residual connections help train deeper GNNs
- mean aggregation normalizes neighbor contribution
- Kipf normalization uses symmetric degree normalization
- max pooling aggregation applies a learned transform to neighbors and takes maxima
- attention aggregation learns data-dependent weights for neighbors

Graph attention resembles transformer attention but is usually restricted to graph edges rather than all token pairs.

Too many layers can cause oversmoothing: node embeddings in connected regions become too similar, making node-level discrimination difficult.

== Edge Graphs

Some problems require reasoning about edges themselves. An edge graph or line graph converts original edges into nodes, with connections between edge-nodes when the original edges share an endpoint. This allows node-style message passing over relationships between edges.

== Notes

Spectral GNN methods use graph Laplacians and graph Fourier ideas. Spatial methods define local aggregation directly in the graph neighborhood. Modern message-passing GNNs are usually spatial.

GraphSAGE is an inductive framework that samples neighborhoods and aggregates their representations. Attention-based GNNs learn which neighbors matter most for each update.

== Takeaway

Graph neural networks extend shared local processing to irregular relational data. They aggregate neighbor information with permutation-respecting operations, build wider graph receptive fields with depth, and use invariant readouts for graph-level predictions.
