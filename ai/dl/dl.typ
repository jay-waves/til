#import "../../appx/theme.typ" as theme
#show: theme.template

#let argmin = math.op("arg min", limits: true)
#let vec(x) = math.bold(math.upright(x))

= supervised learning 

$ vec(y)=f[vec(x),phi] $ 

We learn the parameters $phi$ from *training dataset* of pairs of intput and output examples $x_i, y_i$ , in order to minimize the *loss function*: 

#let argmin = math.op("arg min", limits: true)
#let vphi = $phi.alt$

$ hat(vphi) = argmin_vphi L[vphi] $

After training, we run the model on _separate test data_ to see how well it generalizes 
to exmpales that it didn't observe during training.

== shallow neural networks 

$
y &= f[x, vphi] \
& = vphi_0 + vphi_1 h_1 +vphi_2 h_2 + vphi_3 h_3
$ <eq:shallow>

其中：

We refer to $h_i$ as _hidden units_:

$
h_1 &= upright(a) [theta_(1 0) + theta_(1 1)x] \
h_2 &= upright(a) [theta_(2 0) + theta_(2 1)x] \
h_3 &= upright(a) [theta_(3 0) + theta_(3 1)x] 
$


The most common choice of _activation function_ $upright(a)[dot]$ is the _rectified linear unit (ReLU)_. Activation functions are necessary for *nonlinearity*. 

$
upright(a)[x]=upright(R e L U)[x ]=cases(
0 & x < 0, x & x >= 0
)
$

#figure(
  image("../../assets/ai/shallow-neural-network.webp", width: 40%),
  caption: [Illu of @eq:shallow]
)


