#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 11],
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

= PDF 12: Residual Networks

== Motivation

Image classification improved when CNNs grew from AlexNet to VGG, but adding many more sequential layers eventually degraded both training and test performance. Since training error also worsened, the issue was optimization, not just generalization.

Residual blocks change what each layer learns: instead of directly transforming the representation, a block learns an additive change to it. Batch normalization is then used to control the activation and gradient magnitudes introduced by repeated additions.

== Sequential Processing

A standard network processes data layer by layer:

$ h_1 = f_1(x, phi_1) $

$ h_2 = f_2(h_1, phi_2) $

$ h_3 = f_3(h_2, phi_3) $

$ y = f_4(h_3, phi_4) $

Equivalently, it is a nested composition:

$ y = f_4(f_3(f_2(f_1(x, phi_1), phi_2), phi_3), phi_4) $

In principle, adding layers should not hurt because a deeper network could learn identity-like transformations. In practice, plain deep networks can be hard to optimize.

One proposed explanation is shattered gradients. In deep plain networks, a tiny change in early representations can make the gradient change unpredictably. Gradient descent assumes nearby points have related gradients; shattered gradients break this assumption.

== Residual Block

Residual or skip connections add a block's input back to its processed output:

$ h_1 = x + f_1(x, phi_1) $

$ h_2 = h_1 + f_2(h_1, phi_2) $

$ h_3 = h_2 + f_3(h_2, phi_3) $

$ y = h_3 + f_4(h_3, phi_4) $

Each function learns an additive correction. If $f_k$ is zero, the block is exactly identity. This makes it easy for a deep model to behave like a shallower one.

The input and output of a residual block must have the same shape unless a projection or channel-padding operation is used to match dimensions.

== Why Residual Connections Help

Unraveling a residual network shows that the output is a sum of many paths of different lengths. Some paths pass through many transformations; others are short skip routes.

For a simple block:

$ (partial y) / (partial x) = I + (partial f) / (partial x) $

The identity term gives gradients a direct route. In deeper residual networks, each layer contributes through both short and long paths, so gradients are less likely to be shattered by a single long chain of derivatives.

Residual networks can also be interpreted as ensembles of many shallower subnetworks whose outputs are summed.

== Order of Operations

A typical layer applies a linear transform or convolution and then ReLU. In a residual block, ending with ReLU would make the residual branch non-negative, so the block could only add non-negative corrections.

Residual blocks usually put activation before the final linear operation so the branch can add positive or negative corrections. Practical residual blocks may contain several operations and usually end with a linear or convolutional transform.

Networks often start with a non-residual linear or convolutional layer before the first residual block, so negative raw inputs are not immediately clipped away.

== Variance Growth in Residual Networks

He initialization stabilizes variance through ordinary ReLU layers, but residual addition creates a new problem. If the residual branch has roughly the same variance as its input and is uncorrelated with it, adding them doubles variance.

Thus residual networks avoid vanishing signals but can suffer exponential activation growth and exploding gradients with depth.

One theoretical fix is scaling each residual sum by $1 / sqrt(2)$. In practice, residual architectures usually use batch normalization.

== Batch Normalization

BatchNorm standardizes activations across the batch, then learns a scale and shift. For activation values $h_i$ in batch $B$:

$ m_h = 1 / |B| sum_(i in B) h_i $

$ s_h = sqrt(1 / |B| sum_(i in B) (h_i - m_h)^2) $

Standardize:

$ h_i arrow (h_i - m_h) / (s_h + epsilon) $

Then scale and shift:

$ h_i arrow gamma h_i + delta $

The learned $gamma$ and $delta$ set the post-normalization standard deviation and mean. In convolutional networks, statistics are usually computed over batch and spatial positions for each channel.

At inference, batch statistics are unavailable, so stored training-set running statistics are used.

BatchNorm provides several benefits:

- stabilizes forward propagation
- makes residual-network variance grow more slowly at initialization
- smooths optimization enough to permit larger learning rates
- injects minibatch-dependent noise, which can regularize

It also adds parameters and creates redundancy, because rescaling preceding weights can be canceled by normalization.

== ResNet

ResNets apply residual blocks to convolutional image classifiers. A standard block contains BatchNorm, activation, convolution, then another BatchNorm, activation, convolution, followed by addition to the input.

Bottleneck residual blocks reduce parameter count with three convolutions:

1. 1 by 1 convolution reduces channels
2. 3 by 3 convolution processes the smaller representation
3. 1 by 1 convolution restores channels

ResNet-200 uses many bottleneck residual blocks with periodic downsampling and channel increases. It significantly improved ImageNet performance compared with AlexNet and VGG.

== DenseNet

DenseNet uses concatenation rather than addition. Each layer receives the concatenated outputs of all previous layers, processes them, and appends its new representation to the growing channel stack.

This gives direct access to earlier features and stable gradient paths. The cost is channel growth, so DenseNet uses bottleneck 1 by 1 convolutions and breaks concatenation chains at downsampling points.

DenseNet can be more parameter-efficient than ResNet because it reuses earlier computations flexibly.

== U-Net and Hourglass Networks

Encoder-decoder networks downsample to integrate global context and then upsample to produce dense outputs. The bottleneck may lose high-resolution detail.

U-Net fixes this by concatenating encoder representations to decoder representations at matching spatial scales. This passes fine detail around the bottleneck. Original U-Net used valid convolutions and cropped encoder features before concatenation; later versions often use padding.

U-Net is fully convolutional, so it can run on images of different sizes after training. It was introduced for medical image segmentation and is now widely used, including as a component in diffusion models.

Hourglass networks are similar encoder-decoder models, often with processed skip links that add rather than concatenate. Stacked hourglass networks repeatedly move between local and global representations and have been used for pose estimation with heatmap outputs.

== Why Residual Models Work Well

Residual connections do more than make very deep networks trainable. Shallower wider residual networks can outperform deeper narrower ones with similar parameter counts, and gradients may not effectively use extremely long paths.

A current view is that residual connections smooth the loss surface and make optimization more robust. They create short paths, preserve earlier representations, and bias the model toward learnable corrections.

== Notes

ResNet v1 placed processing before addition; pre-activation ResNet v2 puts BatchNorm and activation before convolutions. Many later architectures adapt residual ideas, including highway networks, DenseNet, U-Net, and transformer blocks.

BatchNorm is not the only normalization method. LayerNorm, InstanceNorm, and GroupNorm normalize across different axes and are useful when batch statistics are unreliable, such as very small batches or sequence models.

== Takeaway

Residual networks make depth trainable by learning additive corrections, supplying short gradient paths, and using normalization to control the variance growth introduced by repeated residual additions.
