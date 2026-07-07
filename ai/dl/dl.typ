
#let argmin = math.op("arg min", limits: true)
#let vec(x) = math.bold(math.upright(x))


== Deep Learning

* Graph Neural Network  
* Transformer 
* Recurrent Neual Network (RNN)
* Convolutional network

== supervised learning 

$ vec(y)=f[vec(x),phi] $ 

We learn the parameters $phi$ from *training dataset* of pairs of intput and output examples $x_i, y_i$ , in order to minimize the *loss function*: 

$ hat(phi) = argmin_phi L[phi] $

==== shallow neural networks 

$
y &= f[x, phi] \
& = phi_0 + phi_1 h_1 + phi_2 h_2 + phi_3 h_3
$

其中：

$
h_1 &= upright(a) [theta_(1 0) + theta_(1 1)x] \
h_2 &= upright(a) [theta_(2 0) + theta_(2 1)x] \
h_3 &= upright(a) [theta_(3 0) + theta_(3 1)x] 
$


The most common choice of *activation function* $upright(a)[dot]$ is the *rectified linear unit* (ReLU). Activation functions are necessary for *nonlinearity*. 

$
upright(a)[x]=upright(R e L U)[x ]=cases(
0 & x < 0, x & x >= 0
)
$

![udl-fig3.4|400](http://oss.jay-waves.cn/til/shallow-neural-network.avif)

