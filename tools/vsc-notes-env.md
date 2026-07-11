
## Markdown

### Plugins

* Markdown All in one: 主要的 LPS 
* Markdown yaml Preamble: 用于文档前的属性
* Markdown Footnotes: 用于脚标支持
* Bookmarks: 用于管理工作流

### Preview 

VSCode Markdwon Preview 的样式，可以用配置项 `markdown.styles` 指定。我使用了两种：
* [github-markdown.css](https://github.com/sindresorhus/github-markdown-css/blob/main/github-markdown.css) 用于正文。`body.vscode-dark` 类可以用于选中深色模式。
* [hlhs - github-dark.css](https://raw.githubusercontent.com/highlightjs/highlight.js/refs/heads/main/src/styles/github-dark-dimmed.css) 用于代码块，大部分样式选择器类似 `highlight.js` ，但是根 class 不太一样。

如果懒得配置可以直接用 GitHub Markdown Preview 插件

## Typst 

模板详见 `../appx/theme.typ`

### Libraries

`appx/theme.typ` 依赖这些 Typst Universe 包:

* `@preview/ctheorems:1.1.3`: 定理、引理、推论、定义、证明环境。
* `@preview/physica:0.9.8`: 科学/工程数学符号，如导数、偏导、梯度、旋度、张量指标等。通过 `theme.typ` 导出为 `physica` 命名空间，示例: `#import "../../appx/theme.typ": tufte, physica`。

### Plugins 

* 需要独立的 [Typst 编译器](https://github.com/typst/typst/releases)
* Tinymist Typst: 主要的 LPS 
* Typst Math
* vscode-pdf

### Dark/Light

依赖 `tynymist` 自动注入的 [`x-preview` 变量](https://myriad-dreamin.github.io/tinymist/feature/preview.html#label-sys.inputs) 来检测系统明暗。

