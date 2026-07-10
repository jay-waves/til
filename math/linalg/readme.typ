#import "../../appx/theme.typ": tufte, meta

#show: tufte

#meta[
- subtitle: 矩阵关系
]

== 矩阵的关系

#table(columns: 5,
  [矩阵关系],
  [定义域],
  [定义],
  [代表],
  [不变量],
  [#link("矩阵等价.typ")[等价] (相抵)],
  [$R_(m times n)tilde.equiv R_(m times n)$],
  [对于初等矩阵 $P,Q$, 有 $B = P_(1)... P_(s)A Q_(1)... Q_(t)$],
  [相抵标准型 $E^((r))_(m times n)$],
  [秩],
  [#link("矩阵相似.typ")[相似]],
  [$R_(n times n)tilde R_(n times n)$],
  [存在可逆矩阵 $P$, 有 $B=P^(-1)A P$],
  [相似对角矩阵 $Lambda = upright("diag") (lambda_(1),..., lambda_(k))$],
  [特征值, 秩],
  [#link("mat-congruence.md")[合同]  (相合)],
  [$R_(n times n)tilde.eq R_(n times n)$],
  [存在可逆矩阵 $P$, 有 $B=P^(top)A P$],
  [合同标准型 $Lambda = upright("diag") (1,...,1,- 1,...,- 1,0,...)$],
  [惯性指数, 秩],
)


#image("../../attach/mat-relation.webp", width: 60%)

== 矩阵的分解

详见 #link("mat-decompose.typ")[矩阵分解]

== 参考

#table(columns: 6,
  [TITLE],
  [AUTHORS],
  [PUBL / CONF / JOUR],
  [YEAR],
  [VOLUME],
  [URL / DOI / ISBN],
  [The Art of Linear Algebra],
  [Kenji Hiranabe],
  [GitHub],
  [2021],
  [],
  [#link("https://github.com/kenjihiranabe/The-Art-of-Linear-Algebra")],
  [Linear Algebra for Everyone],
  [Gilbert Strang],
  [Wellesley-Cambridge Press],
  [2020],
  [],
  [ISBN: 978-1-7331466-3-0],
  [Linear Algebra Done Right],
  [Sheldon Axler],
  [Springer],
  [2015],
  [3rd ed.],
  [ISBN: 978-3-319-11079-0],
  [线性代数],
  [李尚志],
  [高等教育出版社],
  [2004],
  [4th ed.],
  [ISBN: 978-7-04-014815-5],
)
