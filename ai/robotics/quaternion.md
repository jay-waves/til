$$\newcommand{\zm}{\lvert z\rvert}$$

$$\newcommand{\RM}{\begin{bmatrix}
\cos \theta & -\sin \theta \\
\sin \theta & \cos \theta
\end{bmatrix}}$$

$$
\newcommand{\dd}{\,\mathrm{{d}}}% 积分符号
\newcommand{\pfrac}[2]{\frac{\partial #1}{\partial #2}} % 偏导
\newcommand{\R}{\mathbb{R}}
\newcommand{\C}{\mathbb{C}}
\newcommand{\N}{\mathbb{N}}
\newcommand{\Np}{\mathbb{N}^{+}}
\newcommand{\Z}{\mathbb{Z}}
\newcommand{\Q}{\mathbb{Q}}
\newcommand{\ie}{\textit{i.e.}\ }     % 即
\newcommand{\eg}{\textit{e.g.}\ }     % 例如
\newcommand{\etal}{\textit{et al.}\ } % 等
\newcommand{\Conj}[1]{\overline{#1}}       % 共轭 
\newcommand{\Norm}[1]{\lvert #1\rvert}       % 范数
\newcommand{\Ree}[1]{\Re(#1)} % 实部
\newcommand{\Imm}[1]{\Im(#1)} % 虚部
\newcommand{\BigO}[1]{\mathcal{O}\!\left(#1\right)}
$$

$$
\newcommand{\Vec}[1]{\mathbf{#1}} % 向量
\newcommand{\M}[1]{\mathbf{#1}} % 矩阵字母
\newcommand{\TrpM}[1]{\mathbf{#1}^\mathsf{T}} % 转置矩阵
\newcommand{\Trp}{^\mathsf{T}} % 转置矩阵
\newcommand{\InvM}[1]{\mathbf{#1}^{-1}} % 矩阵字母
\DeclareMathOperator{\Tr}{Tr} % 矩阵迹
\DeclareMathOperator{\Rank}{Rank} % 矩阵迹
\DeclareMathOperator{\Diag}{Diag} % 矩阵迹
\DeclareMathOperator{\Sgn}{Sgn} % 矩阵迹
\DeclareMathOperator{\Erf}{Erf} % 矩阵迹
$$


### 复数

复数 $a+bi$ 同构映射于矩阵形式： $$\begin{bmatrix}
a & -b \\
b & a
\end{bmatrix}$$

如 $$i\longleftrightarrow \begin{pmatrix}0 & -1\\ 1 & 0\end{pmatrix}$$

记复平面 $z=a+bi$ 与 $\Re$ 的正方向夹角为 $\theta$ ，那么： $$\cos \theta=\frac{a}{\sqrt{ a^{2}+b^{2} }}$$ $$\sin \theta =\frac{b}{\sqrt{ a^{2}+b^{2}}}$$

记 $\zm=\sqrt{ a^{2}+b^{2} }$ ，复数的同构矩阵可表示为*缩放矩阵*与*[旋转矩阵](../../math/calculus/复数.md)*的乘积：$$z=\zm \cdot I\cdot \RM=\begin{bmatrix}\zm  & 0 \\
0 & \zm
\end{bmatrix} \cdot \RM=\zm e^{i\theta}$$ 

因此，一个复数可以表达一次旋转与缩放的线性变换。

### 轴角式旋转

设经过原点的旋转轴 $\Vec{u}=(x,y,z)\Trp$ ，给定向量 ${} \Vec{v} {}$ ，使其沿着旋转轴转动 $\theta$ 角度，得到 $\Vec{v}'$ 。这里使用右手系统来定义旋转正方向。

