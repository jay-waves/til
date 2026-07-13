#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 12],
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

= PDF 13: Transformers

== Motivation

Transformers were introduced for natural language processing, where inputs are sequences of high-dimensional word or subword embeddings. Text has two properties that make fully connected networks unsuitable:

- sequences can be very long and vary in length
- the same word or pattern can appear at many positions, so parameters should be shared

Language also requires content-dependent interactions. In the example sentence from the chapter, pronouns such as "it" and "their" must connect to the words they refer to, even across long spans. Transformers implement these connections with dot-product self-attention.

== Dot-Product Self-Attention

A self-attention block takes $N$ input vectors $x_1, dots, x_N$, each of dimension $D$, and returns $N$ output vectors. First, it computes values:

$ v_m = beta_v + Omega_v x_m $

Each output is a weighted sum of all values:

$ "sa"_n(x_1, dots, x_N) = sum_(m=1)^N a(x_m, x_n) v_m $

The weights $a(x_m, x_n)$ are non-negative and sum to one over $m$ for each output token $n$. Thus self-attention routes different mixtures of the same values to different output positions.

== Query, Key, and Value

Attention weights are computed using queries and keys:

$ q_n = Omega_q x_n, quad k_n = Omega_k x_n, quad v_n = Omega_v x_n $

Queries represent what the receiving token is looking for. Keys represent what source tokens offer. Values contain the information that will be aggregated.

The dot product between a query and a key measures similarity:

$ s_(n m) = k_m^T q_n $

The softmax makes keys compete:

$ a(x_m, x_n) = exp(k_m^T q_n) / sum_(m'=1)^N exp(k_(m')^T q_n) $

The mechanism is nonlinear even though it uses linear value maps, because the weights are nonlinear functions of the input.

== Matrix Form

With token embeddings as columns of $X$:

$ Q = beta_q + Omega_q X $

$ K = beta_k + Omega_k X $

$ V = beta_v + Omega_v X $

The attention weights are computed from $K^T Q$, and the outputs are:

$ "SelfAttention"(X) = V dot softmax(K^T Q) $

The softmax is applied columnwise so each query gets a distribution over all keys.

== Positional Encoding

Self-attention without position information is permutation equivariant. If token order is permuted, the outputs are permuted the same way, so the model cannot distinguish word order.

Transformers add positional information to each token. The original transformer used fixed sinusoidal positional encodings; many modern models use learned or relative positional encodings.

== Scaled Dot-Product Attention

If query and key dimension $D_k$ is large, dot products can have large variance. Large logits make the softmax saturated, which weakens gradients. Scaled attention divides by $sqrt(D_k)$:

$ s_(n m) = (q_n^T k_m) / sqrt(D_k) $

Then:

$ a_(n m) = exp(s_(n m)) / sum_l exp(s_(n l)) $

== Multi-Head Attention

Multiple attention heads run in parallel with different learned query, key, and value projections. Each head can attend to different relationships. The head outputs are concatenated and mixed with another learned linear projection.

Multi-head attention is valuable because a single attention distribution may be too limited: one token may need to collect syntactic, semantic, positional, and task-specific information simultaneously.

== Transformer Layer

A transformer layer alternates:

- attention, which mixes information across tokens
- a per-token MLP, which transforms each token representation independently

Both sublayers are usually wrapped with residual connections and normalization:

$ X arrow X + "SelfAttention"("Norm"(X)) $

$ X arrow X + "MLP"("Norm"(X)) $

The exact ordering of normalization varies across implementations.

== Text Pipeline

Natural language text must first be tokenized. Modern systems use subword tokenization so rare words can be represented by combinations of common fragments while keeping the vocabulary finite.

Each token is mapped to an embedding vector, positional information is added, and the resulting matrix is processed by a stack of transformer layers.

== Encoder Models: BERT

Encoder models use bidirectional attention: every token can attend to every other token. They are suited to representation learning and tasks such as classification, retrieval, and span prediction.

BERT-like models are pretrained with masked token prediction: some input tokens are hidden, and the model predicts the missing tokens from context. After pretraining, a task-specific head is added and the model is fine-tuned on labeled data.

== Decoder Models: GPT

Decoder models are trained for language modeling: predict the next token from previous tokens. They use causal masked self-attention. Future-token attention scores are set to $-infinity$ before the softmax so each token can only use earlier context.

Generation is autoregressive. The model predicts a distribution for the next token, a token is selected, appended to the context, and the process repeats.

Large decoder models can perform few-shot or in-context learning: the prompt contains examples or instructions, and the model conditions on them without changing parameters.

== Encoder-Decoder Models

Encoder-decoder transformers are used for conditional generation such as machine translation. The encoder processes the source sequence with bidirectional attention. The decoder generates the target sequence causally.

Cross-attention connects them: decoder queries attend to encoder keys and values. This lets each generated token retrieve relevant source information.

== Long Sequences

The attention cost is quadratic in sequence length:

$ "cost" prop N^2 D $

because full self-attention forms an $N times N$ interaction matrix. Long-context methods reduce cost with local attention, sparse patterns, recurrence, memory, low-rank approximations, or chunking.

== Vision Transformers

Transformers can also process images by converting pixels or patches into tokens.

ImageGPT models pixels autoregressively but is expensive because each pixel or small group of pixels is a token. Vision Transformer (ViT) reduces sequence length by splitting the image into fixed-size patches, embedding each patch, adding position encodings, and applying a transformer encoder.

ViT operates on a fixed patch grid and has less built-in locality than CNNs. Multi-scale and shifted-window transformers reintroduce some locality and hierarchy by attending within windows and shifting those windows across layers.

== Notes

Transformers replaced recurrent neural networks in many NLP applications because self-attention allows direct long-range interactions and parallel computation across sequence positions.

Transformers are also related to graph neural networks. Full self-attention can be viewed as message passing on a complete graph whose edge weights depend on content.

The core architectural idea is content-based routing: decide which tokens exchange information based on learned similarities, then refine each token independently with MLPs.

== Takeaway

Transformers process sequences with shared, content-dependent attention. Queries and keys compute routing weights, values carry information, positional encodings restore order, and stacked attention/MLP layers support encoder, decoder, encoder-decoder, and vision architectures.
