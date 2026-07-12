* `==二级标题级别`
* `_斜体_`
* `*粗体*·
* `` `行内代码` ``
* `<标签>`
* `@引用标签或bib`
* `-无序列表`
* `+有序列表`
* `#图片` 
* `#表格` 
* `$公式$` `$ 行间公式 $`

typst 中使用 `#` 前缀来标记命令（表达式）。

```typst
#rect(
  width: 2cm,
  height: 1cm,
  stroke: red,
)
```

## 图片

```typst
#image("a.png", width: 60%)

#figure(
    image("a.png"),
    caption: [图注],
)
```

不过 typst 不允许加载网络图片，这一点比较坑。另一点是文件间相互引用不好。

## 表格 

$2\times 3$ Table:

```typst
#let rows = (
  (
    [r1c1],
    [r1c2],
    [r1c3],
  ),
  (
    [r2c1],
    [r2c2],
    [r2c3],
  ),
)

#table(
  columns: 3,
  table.header(
    [c1],
    [c2],
    [c3],
  ),
  ..rows.flatten(),
)
```

## 其他标记命令

- `#hightlight[]`
- `#underline[]`
- `#strike[]`
- `#sub[]`
- `#super[]`

## 页面命令

- `#align(center)[xxx]`
- `#box(inset: 4pt)` 行内容器
- `#block(width: 80%)` 行间容器（块级）
- `#h(1em)`  水平空白
- `#v(1em)` 垂直h空白
- `#linebreak()`
- `#pagebreak()`
- `#colbreak()`

## 绘图

* [gribouille 0.4.1](https://github.com/mcanouil/gribouille)
* ctez