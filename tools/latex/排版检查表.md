## 符号和单位

- 单引号是 `` `你好'``
- 双引号是 ` ``你好'' `
- 使用 `\ldots` 代替 `...`, 避免字符间距不匀
- 破折号:
	- `-` 连接符号
	- `--` 范围, 如 1--10
	- `---` 句中破折号, 表意义延申

## 排版细节

- 避免孤行和寡头:
```latex
\widowpenalty=10000
\clubpenalty=10000
```
- 防止标题出现在页面底部
```latex
\usepackage{titlesec}
\titlespacing*{\section}{0pt}{*4}{*2}
```
- 页码从正文开始:
```latex
\pagenumbering{arabic}
```
- 避免页内过多图 
- 表格放在页首 `htop` --> `top`