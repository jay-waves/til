latex 提倡 structure 和 style 应该分开, cls 提供 structure (如各种命令和环境) kaobook.cls 提供:
- page layout
- chapter headings
- page headers
- matters
- margin text
- matgin figs/tabs
- margin toc
- hyperef
- bibliography

其他 style 文件 (.sty), kaobook 只提供默认设置, 需要自己改.

kaobook templates 整体结构:

```
kaobook/
|-- kaobook.cls                 - book-specific definitions
|-- kaohandt.cls                - handout-specific definitions
|-- kao.sty	    				- main definitions
|-- kaobiblio.sty	    		- style of the bibliography
|-- kaotheorems.sty             - colorful styling of theorems
`-- kaorefs.sty                 - commands for referencing
```

compile commands: 
```bash
pdflatex main # compile
makeindex main.nlo -s nomencl.ist -o main.nls
makeindex main # compile index
biber main # compile bibliography
makeglossaries main # compile glossary
pdflatex main # compile template again
pdflatex main # compile template again
```

There are defined two page layouts, margin and wide, and two page
styles, plain and fancy. The layout basically concern the width of the margins, while the style refers to headers and footer


## Main Stuff

**sidenotes** inserts contents into side margins. type: `floats`.

packages: `sidenotes` -> `marginnote` -> `marginfix, placeins`

```latex
\sidenote[mark][offset]{Text}
\marginnote[-12pt]{Text}
```

**margintoc** (toc, table of contents, 目录) put small table of contents in margin.

```latex
\setchapterpreamble[u]{\margintoc}
\chapter{Chapter title}
```

**marginfigure, margintable**: usually, figure captions are below, while table captions are above.

```latex
\begin{marginfigure}[offset]
	\includegraphics{monalisa}
	\caption[The Mona Lisa]{The Mona Lisa.}
	\labfig{marginmonalisa}
\end{marginfigure}
```

**widefigure, widetable**

**longtable** 

## Reference

`\sidecite` + `\formatmargincitation`, in `packages.sty`

```latex
\usepackage[style=philosophy-modern]{styles/kaobiblio}
\renewcommand{\formatmargincitation}[1]{
	\citeyear{#1}, \citeauthor*{#1}: \citetitle{#1}; very interesting!%
}
\addbibresource{main.bib}
```

**glossary** whenever the glossries are changed, remake with:

```bash
> pdflatex main
> makeglossaries main
> pdflatex main
```

**indice**

**nomenclature**

```latex
\nomenclature
\printnomenclature
```

compile nomenclature:

```bash
> pdflatex
> makeindex main.nlo -s nomencl.ist -o main.nls
> pdflatex main
```

### compile

latexmk configuration:
```lua
@default_fiels=('main.tex');

add_cus_dep('acn', 'acr', 0, 'makeglossaries');
add_cus_dep('glo', 'gls', 0, 'makeglossaries');
$clean_ext .= "acr acn alg glo gls glg";
sub makeglossaries {
	my ($base_name, $path) = fileparse( $_[0] );
	pushd $path;
	my $return = system "makeglossaries", $base_name
	popd;
	return $return;
}

add_cus_dep('nlo'', 'nls', 0, 'makenlo2nls');
sub makenlo2nls {
	system( "makeindex -s nomencl.ist -o \"$_[0].nls\" \"$_[0].nlo\"");
}
```

## Page Design

### Headings

chapter title style:　`plain, kao, bar, lines`

```latex
\setchapterstyle{kao} % head 风格
\setchapterpreamble[u]{\margintoc} % margin 小目录
\chapter{ Tiltle of this chapter }
\labch{title} % 标签

% 风格2
\setchapterimage[7cm]{path/to/img.avif} % 会自动将风格应用为 bar
\setchapterpreamble[u]{\margintoc}
\chapter{Catchy Title} % No need to set a chapter style
\labch{catchy}
```

### TOC

| entry                 | command                                      |
| --------------------- | -------------------------------------------- |
| table of contents     | `\setuptoc{toc}{totoc}`                        |
| list of figs and tabs | `\PassOptionsToClass{toc=listof}{\@baseclass}` |
| bibilography          | `\PassOptionsToClass{toc=bibliography}{\@baseclass}`                                             |

### Page Layout

```latex
\renewcommand{\marginlayout}{%
	\newgeometry{
		top            = 27.4 mm, % height of top margin
		bottom         = 27.4 mm, % height of bottom margin
		inner          = 24.8 mm, % width of inner margin
		textwidth      = 117  mm, % width of the text
		marginparsep   = 8.2  mm, % width between text and margin
		marginparwidth = 39.4 mm, % width of the margin
		
	}
}
```

## Mathematica & Box

definitions, example and remarks have independent counters; theorems, propositions, lemmas and corollaries share the same counter.

`[framed:background=mycolour]`


code: