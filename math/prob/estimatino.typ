#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

#let group(title) = table.cell(
  colspan: 5,
  align: center,
  strong(title),
)

#let xb  = $macron(X)$
#let yb  = $macron(Y)$
#let dxy = $xb - yb$

#let sez = $sqrt(sigma_1^2 / n_1 + sigma_2^2 / n_2)$
#let sep = $S_W sqrt(1 / n_1 + 1 / n_2)$
#let df  = $n_1 + n_2 - 2$

#let r1 = (
  [$mu$],
  [$sigma^2$ known],
  [$Z = frac(xb - mu, sigma / sqrt(n)) ~ N(0, 1)$],
  [$xb plus.minus sigma / sqrt(n) z_(alpha / 2)$],
  [
    $overline(mu) = xb + sigma / sqrt(n) z_alpha$ \
    $underline(mu) = xb - sigma / sqrt(n) z_alpha$
  ],
)

#let r2 = (
  [$mu$],
  [$sigma^2$ unknown],
  [$t = frac(xb - mu, S / sqrt(n)) ~ t(n - 1)$],
  [$xb plus.minus S / sqrt(n) t_(alpha / 2)(n - 1)$],
  [
    $overline(mu) = xb + S / sqrt(n) t_alpha(n - 1)$ \
    $underline(mu) = xb - S / sqrt(n) t_alpha(n - 1)$
  ],
)

#let r3 = (
  [$sigma^2$],
  [$mu$ unknown],
  [
    $chi^2 = frac((n - 1) S^2, sigma^2) ~ chi^2(n - 1)$
  ],
  [ #block[
    $(
      frac((n - 1) S^2, chi_(alpha / 2)^2(n - 1)),
      frac((n - 1) S^2, chi_(1 - alpha / 2)^2(n - 1)),
    )$ 
  ]],
  [
    $overline(sigma^2) = frac((n - 1) S^2, chi_(1 - alpha)^2(n - 1))$ \
    $underline(sigma^2) = frac((n - 1) S^2, chi_alpha^2(n - 1))$
  ],
)

#let r4 = (
  [$mu_1 - mu_2$],
  [$sigma_1^2, sigma_2^2$ known],
  [
    $Z = frac(dxy - (mu_1 - mu_2), sez) ~ N(0, 1)$ ],
  [
    $dxy plus.minus z_(alpha / 2) sez$
  ],
  [
    $overline(mu_1 - mu_2) = dxy + z_alpha sez$ \ #v(6pt)
    $underline(mu_1 - mu_2) = dxy - z_alpha sez$
  ],
)

#let r5 = (
  [$mu_1 - mu_2$],
  [$sigma_1^2 = sigma_2^2 = sigma^2$ unknown],
  [
    $t = frac(dxy - (mu_1 - mu_2), sep) \ quad ~ t(df)$ \
    $S_W^2 = frac( (n_1 - 1) S_1^2 + (n_2 - 1) S_2^2, df,)$
  ],
  [
    $dxy \ plus.minus t_(alpha / 2)(df) sep$
  ],
  [
    $overline(mu_1 - mu_2) = dxy \ quad + t_alpha(df) sep$ \ #v(6pt)
    $underline(mu_1 - mu_2) = dxy \ quad - t_alpha(df) sep$
  ],
)

#let ssdiv = $S_1^2 slash S_2^2$
#let iidiv = $sigma_1^2 slash sigma_2^2$

#let r6 = (
  [$frac(sigma_1^2, sigma_2^2)$],
  [$mu_1, mu_2$ unknown],
  [
    $F = frac( ssdiv, iidiv) \ quad ~ F(n_1 - 1, n_2 - 1)$
  ],
  [ #block[
    $( frac( ssdiv, F_(alpha / 2)(n_1 - 1, n_2 - 1),), 
    frac( ssdiv, F_(1 - alpha / 2)(n_1 - 1, n_2 - 1),))$ 
  ]],
  [
    $overline(iidiv) = frac( ssdiv, F_(1 - alpha)(n_1 - 1, n_2 - 1),)$ \ #v(6pt)
    $underline(iidiv) = frac( ssdiv, F_alpha(n_1 - 1, n_2 - 1),)$
  ],
)

#pad(right: -40mm)[
#table(
    columns: (5%, 10%, 25%, 30%, 30%),
    inset: (x: 5pt, y: 10pt),
    table.header([待估参数], [其他参数], [样轴量的分布], [置信区间], [单侧置信限],),
    table.hline(),
    group("单个正态总体"),
    ..r1,
    ..r2,
    ..r3,

    group("两个正态总体"),
    ..r4,
    ..r5,
    ..r6,
  )
]


=== 似然度
#strong[似然度 (likelihood)] 用于评估模型中#strong[参数]的可能性. 给定总体 $X$ (独立同分布随机变量), $theta$ 为各个随机变量 $X_i$ 的概率建模 (如概率分布函数) 中的某些参数, 似然度使用总体 $X$ 来估计参数, 结果为一个#strong[估计量] (随机变量) $hat(theta) ( X_1 , X_2 , dots.h , X_n )$

在具体的数据点 ${ x_i }$ 上, $hat(theta) ( x_1 , x_2 , dots.h , x_n )$ 称为参数的具体#strong[估计值].

#strong[最大似然估计] (Maximum Likelihood Estimation, MLE):

通过总体 $X$ 中随机变量的概率密度函数 $f(x_i | theta)$, 计算似然函数: $ L(theta | X) = F(x_1 , x_2 , dots.h , x_n | theta) = product_(i = 1)^n f(x_i | theta) $

最大似然估计要寻找 $hat(theta)$ 估计量使 $L(theta | X)$ 值最大, 此时称该参数使得观测到改组数据 ${ x_i }$ 的可能性最大:

$ hat(theta) = arg max_theta L(theta) $

实际操作中, 由于似然函数的复杂性, 只求导计算函数驻点而不验证极大, 还可以取单调性和原函数一致的对数函数: $ l(theta | X) = ln ( L(theta | X) ) = sum_(i = 1)^n ln ( f(x_i | theta) ) $




