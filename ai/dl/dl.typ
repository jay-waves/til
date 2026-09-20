#import "@local/ypst-template:0.1.0" as theme
#show: theme.template

#let argmin = math.op("arg min", limits: true)
#let vec(x) = math.bold(math.upright(x))

= supervised learning 

$ vec(y)=f[vec(x),phi] $ 

We learn the parameters $phi$ from *training dataset* of pairs of intput and output examples $x_i, y_i$ , 
 to minimize the *loss function*: 

#let argmin = math.op("arg min", limits: true)
#let vphi = $phi.alt$

$ hat(vphi) = argmin_vphi L[vphi] $

After training, we run the model on _separate test data_ to evaluate generalization on
 exmpales that it didn't observe during training.

== shallow neural networks 

$
y &= f[x, vphi] \
& = vphi_0 + vphi_1 h_1 +vphi_2 h_2 + vphi_3 h_3
$ <eq:shallow>

#theme.sidenote[
  We refer to $h_i$ as _hidden units_:
  $
  h_1 &= upright(a) [theta_(1 0) + theta_(1 1)x] \
  h_2 &= upright(a) [theta_(2 0) + theta_(2 1)x] \
  h_3 &= upright(a) [theta_(3 0) + theta_(3 1)x] 
  $
][
  Number of hidden units in a shallow network is called as _network capacity_.
]

The most common choice of _activation function_ $upright(a)[dot]$ is the _rectified linear unit (ReLU)_. 
Activation functions are necessary for *nonlinearity*. 

$
upright(a)[x]=upright(R e L U)[x ]=
cases(
  0 &quad x < 0, 
  x &quad  x >= 0
)
$

#figure(
  image("../../assets/ai/shallow-neural-network.webp", width: 40%),
  caption: [Illu of @eq:shallow]
)

By the *Universal Approximation THeorem*, shallow-neural-network  
can approximate *any continuous funciton on a compact domain arbitrarily well*, provided with 
sufficiently *wide* & single hidden layer + suitable nonlinear activation funciton. 

== deep nueral networks 

- $K$ as the number of layers 
- $D_i$ as the number fo hidden units in $i$ layer
- $beta_k$ as the vector of biases (intercepts) contibuted to hidden layer $k+1$
- $k^"th"$ as the weights (slopes) for the $k$ layer 

$ vec(y) = f[vec(x), vec(phi.alt)],quad vec(phi.alt) = {vec(beta)_i, vec(Omega)_i}^K $

Matrices of $vec(Omega)_i$ are $D_(i+1)times D_i$. Both deep and shallow networks can model 
arbitrary functions, but some functions can be much more efficiently with deep networks.

$
  vec(h_1) &= a[vec(beta_0) + vec(Omega)_0 vec(x)] \
  vec(h_2) &= a[vec(beta_1) + vec(Omega)_1 vec(h)_1] \
  & dots.v \
  vec(h_K) &= a[vec(beta)_(K-1) + vec(Omega)_(K-1) vec(h)_(K-1)]\
  vec(y) &= vec(beta)_K + vec(Omega)_K vec(h)_K
$

#figure(
  image("../../assets/ai/deep-neural-network.webp", width: 60%),
)

