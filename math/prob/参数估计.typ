#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

#table(
    columns: (5.65%, 6.56%, 21.31%, 26.05%, 40.44%),
    align: (auto,auto,auto,auto,auto,),
    table.header([待估参数], [其他参数], [样轴量的分布], [置信区间], [单侧置信限],),
    table.hline(),
    [单个正态总体], [], [], [], [],
    [$mu$], [$sigma^2$ 已知], [$Z = frac(macron(X) - mu, sigma / sqrt(n)) tilde.op N(0 , 1)$], [$macron(X) plus.minus sigma / sqrt(n) z_(alpha / 2)$], [$ overline(mu) = macron(X) + sigma / sqrt(n) z_alpha $ $ underline(mu) = macron(X) - sigma / sqrt(n) z_alpha $],
    [$mu$], [$sigma^2$ 未知], [$t = frac(macron(X) - mu, S / sqrt(n)) tilde.op t(n - 1)$], [$macron(X) plus.minus S / sqrt(n) t_(alpha / 2)(n - 1)$], [$ overline(mu) = macron(X) + S / sqrt(n) t_alpha(n - 1) $ $ underline(mu) = macron(X) - S / sqrt(n) t_alpha(n - 1) $],
    [$sigma^2$], [$ mu $ 未知], [$chi^2 = frac(( n - 1 ) S^2, sigma^2) tilde.op chi^2(n - 1)$], [$lr((frac((n - 1) S^2, chi_(alpha / 2)^2(n - 1)), frac((n - 1) S^2, chi_(1 - alpha / 2)^2(n - 1))))$], [$overline(sigma^2) = frac((n - 1) S^2, chi_(1 - alpha)^2(n - 1))$ $underline(sigma^2) = frac((n - 1) S^2, chi_alpha^2(n - 1))$],
    [两个正态总体], [], [], [], [],
    [$mu_1 - mu_2$], [$sigma_1^2 , sigma_2^2$ 已知], [$Z = frac(macron(X) - macron(Y) - (mu_1 - mu_2), sqrt(frac(sigma_1^2, n_1) + frac(sigma_2^2, n_2))) tilde.op N(0, 1)$], [$macron(X) - macron(Y) plus.minus z_(alpha / 2) sqrt(sigma_1^2 / n_1 + sigma_2^2 / n_2)$], [$ overline(mu_1 - mu_2) = macron(X) - macron(Y) + z_alpha sqrt(sigma_1^2 / n_1 + sigma_2^2 / n_2) $ $ underline(mu_1 - mu_2) = macron(X) - macron(Y) - z_alpha sqrt(sigma_1^2 / n_1 + sigma_2^2 / n_2) $],
    [$mu_1 - mu_2$], [$sigma_1^2 = sigma_2^2 = sigma^2$ 未知], [$t = frac(macron(X) - macron(Y) - (mu_1 - mu_2), S_W sqrt(frac(1, n_1) + frac(1, n_2))) tilde.op t(n_1 + n_2 - 2)$ $S_W^2 = frac((n_1 - 1) S_1^2 + (n_2 - 1) S_2^2, n_1 + n_2 - 2)$], [$macron(X) - macron(Y) plus.minus t_(alpha / 2)(n_1 + n_2 - 2) times S_W sqrt(1 / n_1 + 1 / n_2)$], [$ overline(mu_1 - mu_2) = macron(X) - macron(Y) + t_alpha(n_1 + n_2 - 2) S_W sqrt(1 / n_1 + 1 / n_2) $ $ underline(mu_1 - mu_2) = macron(X) - macron(Y) - t_alpha(n_1 + n_2 - 2) S_W sqrt(1 / n_1 + 1 / n_2) $],
    [$frac(sigma_1^2, sigma_2^2)$], [$mu_1 , mu_2$ 未知], [$F = frac(S_1^2 / S_2^2, sigma_1^2 / sigma_2^2) tilde.op F(n_1 - 1, n_2 - 1)$], [$lr((frac(S_1^2 / S_2^2, F_(alpha / 2)(n_1 - 1, n_2 - 1)), frac(S_1^2 / S_2^2, F_(1 - alpha / 2)(n_1 - 1, n_2 - 1))))$], [$overline(frac(sigma_1^2, sigma_2^2)) = frac(S_1^2 / S_2^2, F_(1 - alpha)(n_1 - 1, n_2 - 1))$ $underline(frac(sigma_1^2, sigma_2^2)) = frac(S_1^2 / S_2^2, F_alpha(n_1 - 1, n_2 - 1))$],
  )

=== 似然度
#strong[似然度 (likelihood)] 用于评估模型中#strong[参数]的可能性. 给定总体 $X$ (独立同分布随机变量), $theta$ 为各个随机变量 $X_i$ 的概率建模 (如概率分布函数) 中的某些参数, 似然度使用总体 $X$ 来估计参数, 结果为一个#strong[估计量] (随机变量) $hat(theta) ( X_1 , X_2 , dots.h , X_n )$

在具体的数据点 ${ x_i }$ 上, $hat(theta) ( x_1 , x_2 , dots.h , x_n )$ 称为参数的具体#strong[估计值].

#strong[最大似然估计] (Maximum Likelihood Estimation, MLE):

通过总体 $X$ 中随机变量的概率密度函数 $f(x_i | theta)$, 计算似然函数: $ L(theta | X) = F(x_1 , x_2 , dots.h , x_n | theta) = product_(i = 1)^n f(x_i | theta) $

最大似然估计要寻找 $hat(theta)$ 估计量使 $L(theta | X)$ 值最大, 此时称该参数使得观测到改组数据 ${ x_i }$ 的可能性最大:

$ hat(theta) = arg max_theta L(theta) $

实际操作中, 由于似然函数的复杂性, 只求导计算函数驻点而不验证极大, 还可以取单调性和原函数一致的对数函数: $ l(theta | X) = ln ( L(theta | X) ) = sum_(i = 1)^n ln ( f(x_i | theta) ) $




