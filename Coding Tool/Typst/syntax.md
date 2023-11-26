heading `=`

italics `_text_`

new paragraph: blank line

numbered list:

```typst
+ item one
+ item two
+ ...
```

bulleted list:
```typst
- item one
- item two
- 
```

image: `#image("/path/to/...")`

figure:
```typst
#figure(
  image("glacier.jpg", width: 70%),
  caption: [
    _Glaciers_ form an important part
    of the earth's climate system.
  ],
)
```

reference: @figure, @headings, @equations -> <label_name>
```typst
Glaciers as the one shown in
@glaciers will cease to exist if
we don't take action soon!

#figure(
  image("glacier.jpg", width: 70%),
  caption: [
    _Glaciers_ form an important part
    of the earth's climate system.
  ],
) <glaciers>
```

bibliography:
```typst
#bibliography("works.bib")
```

## math

inline: `$Q = rho A v + C$`

interline: `$ Q = rho A v + C $`, add single space near `$`

**multiple charaters** will be interpreted as symbols, variables or function. unless surrounded by `""` -> `$ Q = rho A v + "time offset" $`

### symbols:

`$ 7.32 beta + sum_(i=0)^nabla Q_i / 2 $` --> $7.32\beta + \sum^\nabla_{i=0}\frac{Q_i}{2}$

`$ v := vec(x_1, x_2, x_3) $` --> $v := \begin{pmatrix}x_1\\ x_2 \\x_3\end{pmatrix}$