---
tag: QAA, 摘抄
url: https://cs.stackexchange.com/questions/66807/pda-for-xy-x-y-x-%E2%89%A0-y-from-its-grammar-and-intuition-behind-it
---

## PDA for { xy : |x| = |y|, x ≠ y} from its grammar, and intuition behind it

I know the grammar for the language $\{ xy : |x| = |y|, x ≠ y \}$ if $\Sigma=\{a,b\}$:  

$$
\begin{align*}
&S→AB∣BA \\ 
&A→a∣aAa∣aAb∣bAa∣bAb \\
&B→b∣aBa∣aBb∣bBa∣bBb 
\end{align*}
$$

I know this is a grammar, but I need a PDA for this language, and intuition how $\{xy: |x|=|y|,x \neq y\}$I know this is a grammar, but I need a PDA for this language, and intuition how $\{xy: |x|=|y|,x \neq y\}$I know this is a grammar, but I need a PDA for this language, and intuition how $\{xy: |x|=|y|,x \neq y\}$

## Answer:
You are asking two questions: how to construct a PDA for the language, and why this language is context-free while the same language with the condition x≠y replaced by the condition x\=y, is not. I will only answer the second, since for the first question there are known algorithms.

The reason that your language is context-free is that we can rewrite it as follows:
$$
\{ΣiaΣiΣjbΣj:i,j≠0\}∪\{ΣibΣiΣjaΣj:i,j≠0\}.
$$
This gives a different description of the language as the union of concatenations of context-free languages. A similar trick just doesn't work for the language {xy:x\=y}.

Here is a different example. Consider the following two collections of natural numbers (without zero):

*   A\={x⋅y:x≠y}.
*   B\={x⋅y:x\=y}.

The set A consists of all natural numbers other than 1, whereas the set B consists of all squares. Even though we defined them in a similar way, the set A has a much simpler description, while B doesn't.


[Source](https://cs.stackexchange.com/questions/66807/pda-for-xy-x-y-x-%E2%89%A0-y-from-its-grammar-and-intuition-behind-it)