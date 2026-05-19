## Deep Learning

* Graph Neural Network  
* Transformer 
* Recurrent Neual Network (RNN)
* Convolutional network

## supervised learning 

$$\Vec{y}=f[\Vec{x},\phi]$$ 

We learn the parameters $\phi$ from *training dataset* of pairs of intput and output examples ${x_{i},y_{i}}$ , in order to minimize the *loss function*: 

$$\DeclareMathOperator*{\argmin}{arg\,min}$$

$$\hat{\phi}= \argmin_{\phi}{\left[L[\phi]\right]}$$ 

#### shallow neural networks 

$$\begin{align}
y & = f[x,\phi ] \\
&=\phi_{0}+\phi_{1}h_{1}+\phi_{2}h_{2}+\phi_{3}h_{3}
\end{align}$$

$$\begin{align}
h_{1} & =\mathrm{a}[\theta_{10}+\theta_{11}x] \\
h_{2} & =\mathrm{a}[\theta_{20}+\theta_{21}x] \\
h_{3} & =\mathrm{a}[\theta_{30}+\theta_{31}x] \\
\end{align}$$

The most common choice of *activation function* $\mathrm{a}[\cdot]$ is the *rectified linear unit* (ReLU). Activation functions are necessary for **nonlinearity**. 

$$\mathrm{a}[x]=\mathrm{ReLU}[x]=\begin{cases}
0 & x<0  \\
x & x\geq 0
\end{cases}$$

![udl-fig3.4|400](http://oss.jay-waves.cn/til/shallow-neural-network.avif)