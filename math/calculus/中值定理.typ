#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

== 介值定理
, Intermediate Value theorem

$f : [ a , b ] arrow.r bb(R)$ 连续, 且 $f(a) eq.not f(b)$, 则 $forall y in [ f(a) , f(b) ] : exists c in(a , b) : f(c) = y$

== 零点定理
, Zero Point theorem

$f : [ a , b ] arrow.r bb(R)$ 连续, $f(a) dot f(b) < 0$, 则 $exists c in(a , b) : f'(c) = 0$

== 罗尔定理
, Rolle's Theorem

$f : [ a , b ] arrow.r bb(R)$ 是连续的, 在 $( a , b )$ 上可导, 且 $f(a) = f(b)$, 那么 $exists c in(a , b) : f'(c) = 0$

#quote(block: true)[
证明: 用罗尔定理证明拉格朗日和柯西中值定理
]

== 拉格朗日中值定理
, Lagrange's Mean Value Theorem

$f : [ a , b ] arrow.r bb(R)$ 连续, 在 $( a , b )$ 上可导, 则 $exists xi in(a , b) : f'(xi) = frac(f(b) - f(a), b - a)$

== 柯西中值定理
, Cauchy's Mean Value theorem

$f : [ a , b ] arrow.r bb(R)$, $g : [ a , b ] arrow.r bb(R)$ 皆连续, 在 $( a , b )$ 可导, 则 $exists c in(a , b) : frac(f'(xi), g'(xi)) = frac(f(b) - f(a), g(b) - g(a))$




