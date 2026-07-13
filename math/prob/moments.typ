#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

= 概率密度函数

= 数字特征

== 期望

随机变量 $x$ 服从概率分布 $p(x)$，函数 $f(x)$ 的期望定义为

$ E_x[f(x)] = integral f(x) p(x) dif x. $

对于多个随机变量，

$ E_(x,y)[f(x,y)] = integral integral f(x,y) p(x,y) dif x dif y. $

期望可以理解为：按照不同取值出现的概率，对函数值进行加权平均。

若从 $p(x)$ 中独立抽取 $I$ 个样本 $x_1, dots, x_I$，则可用样本均值近似期望：

$ E_x[f(x)] approx 1/I sum_(i=1)^I f(x_i). $

== 均值、方差与协方差

随机变量的*均值*为

$ mu = E[x] $

*方差*描述数据围绕均值的离散程度：

$ sigma^2 = "Var"(x) = E[(x - mu)^2] $

*标准差*为方差的正平方根：

$ sigma = sqrt("Var"(x)). $

两个随机变量 $x$ 和 $y$ 的*协方差*为

$ "Cov"(x,y) = E[(x - mu_x)(y - mu_y)]. $

- 协方差为正：两者通常同向变化。
- 协方差为负：两者通常反向变化。
- 独立必然推出协方差为零。
- 协方差为零不一定意味着独立。

对于随机向量 $bold(x) in RR^D$，*协方差矩阵*为

$ Sigma = E[(bold(x) - bold(mu))(bold(x) - bold(mu))^T]. $

其中第 $(i,j)$ 个元素表示 $x_i$ 与 $x_j$ 的协方差。

== 方差恒等式

方差也可以写成

$ "Var"(x) = E[x^2] - E[x]^2. $

推导如下：

$
E[(x - mu)^2]
&= E[x^2 - 2 mu x + mu^2] \
&= E[x^2] - 2 mu E[x] + mu^2 \
&= E[x^2] - 2 mu^2 + mu^2 \
&= E[x^2] - mu^2 \
&= E[x^2] - E[x]^2.
$

== 众数

== 方差

== 偏度

= 标准化

对均值为 $mu$、标准差为 $sigma$ 的随机变量 $x$，定义

$ z = (x - mu) / sigma. $

标准化后的变量满足

$ E[z] = 0, quad "Var"(z) = 1. $

逆变换为

$ x = mu + sigma z. $

== 多变量标准化

若随机向量 $bold(x)$ 的均值为 $bold(mu)$，协方差矩阵为 $Sigma$，则可进行白化变换：

$ bold(z) = Sigma^(-1/2)(bold(x) - bold(mu)). $

变换后有

$ E[bold(z)] = bold(0), quad "Cov"(bold(z)) = I. $

逆变换为

$ bold(x) = bold(mu) + Sigma^(1/2) bold(z). $