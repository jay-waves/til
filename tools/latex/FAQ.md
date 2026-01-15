---
tags: [FAQ, ]
---

### 如何放大公式?

不同程度: `\large` `\Large` `\LARGE` `\huge` `\Huge` `\big` 

作为修饰符, 修饰整个作用域: `{\large x}`

### 如何输入长竖线?

$$f(x)\Bigg\vert^{\pi}_{-\pi}$$

### 如何手动给公式编号

Markdown 里可以用 `\tag{}` 给行间公式编号.

$$x^{2}=1\tag{1.1}$$

### 如何让角标显示在正上下方?

$\sum_{k=1}$, $\sum\limits_{k=1}$

另外, 用行间公式 `$$$$` 时, 也会自动渲染在求和号的上下.

### 如何调整括号的大小, 使其匹配内容高度?

使用 `\left` 和 `\right` 修饰括号

```latex
\left( \frac{a}{b} \right)
\left[ \frac{a}{b} \right]
\left\{ \frac{a}{b} \right\}
\left| \frac{a}{b} \right|
%%如果括号不匹配, 使用 . 代替另一侧不需要的符号%%
\left\{ \frac{a}{b} \right.
```

### Latex 输入空格

latex多个连续空格会被视为一个, 如下命令可输入多空格:
- `\ `  用反斜杠转义
- `\quad` 产生一个 em 宽度空格
- `\hspace{1cm}` 产生特定长度空格
- `\phatom{text}` 产生和text宽度同的空白空间, 但不显示文本
- `\hfill` 类似 qt 的左 strech, 将文本挤到右侧

| 格式     | 空格大小依次递减 |
| -------- | ---------------- |
| `a\qquad b` |                  |
| `a\quad b`  |                  |
| `a\ b`     |                  |
| `a\;b`     |                  |
| `a\,b`     |                  |
| `ab`     |                  |
| `a\!b`         |                  |

### Latex 输入换行

段内换行: 
- markdown 在行末加两个空格, 
- latex 加 `\\`. 否则默认编译后不换行.

换段: 一个空行, 或 `\par`

### Latex 如何设置中文字体

```latex 
\usepackage{ctex}

\setCJKmainfont{SimSun} % 宋体, 一般系统自带
\setmainfont{Times New Roman} % 英文用 Times New Romain

```

## 伪代码

```latex
\usepackage[ruled,linesnumbered]{algorithm2e} %% ruled 标题在上方, boxed 在盒子里

\begin{algorithm}
\;                       %% 行末分号并换行
\caption{}               %% 标题
\KwData{xxx}             %% Data: xxx
\KwIn{xxx}               %% In: xxx
\KwOut{xxx}              %% Out: xxxx
\KwResult{xxxx}          %% Result: xxxx
\For {cond}{xxx}         %% for <cond> do <xxx> endo
\If {cond}{xxx}          %% if <cond> then <xxx> end
\While {cond} {xxx}      %% while <cond> then <xxx> end 
\tcc{comments}           %% /*commnets*/
\tcp{comments}           %% // comments
\eIf {cond} {xxx} {yyy}  %% if <cond> then <xxx> else <xxx> end
\end{algorithm}
```