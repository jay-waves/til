#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 10],
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

= PDF 11: Convolutional Networks

== Motivation

Fully connected networks are poorly matched to images. Images are high-dimensional, nearby pixels are statistically related, and image interpretation should be stable under transformations such as translation.

Convolutional layers process local regions with shared parameters. This reduces parameter count, exploits local spatial structure, and avoids relearning the same feature detector at every image position. A network mainly built from convolutional layers is a convolutional neural network (CNN).

== Equivariance and Invariance

For a transformation $t$, a function is invariant if:

$ f(t(x)) = f(x) $

Image classification should be approximately invariant: shifting a tree image should not change the class.

A function is equivariant if:

$ f(t(x)) = t(f(x)) $

Semantic segmentation should be equivariant: shifting the image should shift the output labels in the same way.

Convolutional layers are translation equivariant. Pooling, striding, and final aggregation can help build partial invariance.

== 1D Convolution

The chapter introduces convolution in 1D before moving to images. A kernel or filter is a small set of weights reused at every position. With kernel size three:

$ z_i = omega_1 x_(i - 1) + omega_2 x_i + omega_3 x_(i + 1) $

Strictly, this is cross-correlation rather than mathematical convolution because the kernel is not flipped, but this convention is standard in machine learning.

Padding handles boundary positions. Zero padding assumes values outside the input are zero. Valid convolution only computes outputs where the kernel fits entirely inside the input, shrinking the representation.

Stride controls how far the kernel moves between outputs. Stride two produces roughly half as many positions. Kernel size controls the local window width. Dilation inserts gaps between kernel taps, increasing receptive field without adding weights.

== Convolutional Layers

A 1D convolutional layer with kernel size three computes:

$ h_i = a(beta + sum_(j=1)^3 omega_j x_(i + j - 2)) $

This is a special case of a fully connected layer where most weights are zero and the remaining weights are tied across positions. If there are $D$ inputs and $D$ outputs, a fully connected layer has $D^2$ weights and $D$ biases; the convolution above has three weights and one bias.

This parameter sharing is the main inductive bias: the same local computation is applied everywhere.

== Channels

One convolution can lose information, so layers usually compute many convolutions in parallel. Each produces a feature map or channel.

If the input has $C_i$ channels, the output has $C_o$ channels, and kernel size is $K$, then the weights have shape:

$ C_i times C_o times K $

and there are $C_o$ biases. Each output channel combines all input channels over the local spatial window.

== Receptive Field

The receptive field of a hidden unit is the region of the original input that can affect it. With kernel size three, a first-layer unit sees three input positions. A second-layer unit combines nearby first-layer units, so its receptive field is larger. Stacking layers gradually integrates wider context.

The MNIST-1D example shows why convolution helps. A convolutional model with far fewer parameters than a comparable fully connected network generalizes much better, because it is forced to process translated digit templates similarly at every position.

== 2D Convolution

For image data, convolution extends across height and width. A 3 by 3 RGB convolution uses weights over a $3 times 3 times 3$ patch, sums the products, adds a bias, and applies an activation. Sliding the kernel horizontally and vertically creates a 2D hidden layer.

Multiple output channels repeat this operation with different kernels, giving a 3D tensor of hidden units: height, width, and channels.

== Downsampling and Upsampling

Downsampling reduces spatial resolution. Common methods include:

- strided convolution or subsampling
- max pooling over local blocks
- mean pooling over local blocks

Downsampling is applied per channel. Spatial size decreases while channel count often increases.

Upsampling restores spatial resolution for dense prediction. Methods include:

- duplicating values
- max unpooling using saved pooling locations
- bilinear interpolation
- transposed convolution, a learned operation whose matrix form is the transpose of a corresponding downsampling convolution

A 1 by 1 convolution changes channel count without mixing spatial positions. It applies the same fully connected transformation to the channel vector at every position.

== Applications

Image classification maps an image to one class distribution. ImageNet uses 1000 classes and historically drove progress in CNN architecture. AlexNet combined convolutional and fully connected layers and used ReLU, augmentation, dropout, L2 regularization, momentum, and test-time averaging. VGG improved performance mainly by going deeper with repeated small convolutions and pooling.

Object detection identifies multiple objects and their bounding boxes. YOLO divides the image into a grid and predicts class probabilities, bounding box parameters, and confidence values for each grid cell, followed by heuristics to suppress duplicate detections.

Semantic segmentation predicts a class distribution at every pixel. Encoder-decoder networks downsample to integrate context and then upsample to the original resolution. The final layer often uses a 1 by 1 convolution to produce class channels and a softmax per spatial position.

== Architectural Pattern

Typical CNNs interleave convolutional layers with downsampling. As depth increases, spatial dimensions decrease and channel counts increase. Classification models eventually aggregate information across the image with fully connected layers or global pooling. Dense prediction models add a decoder that upsamples back to image resolution.

CNNs improve performance because their equivariance and parameter sharing impose a useful inductive bias. They search a smaller family of plausible image functions than fully connected networks.

== Notes

Convolutional networks are used beyond images, including 1D sequences such as audio and text-like signals and 3D data such as volumetric medical images.

CNNs are often initialized with the same principles as other ReLU networks, though specialized convolutional initializers exist. Regularization and augmentation are central in practical computer vision pipelines.

Increasing CNN depth improved image classification for several years, but simply adding layers eventually made training worse. This motivates residual connections in the next chapter.

== Takeaway

Convolutional networks encode locality, translation equivariance, and parameter sharing. They process local patches similarly across space, gradually expand receptive fields, and use downsampling or upsampling according to the output task.
