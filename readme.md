## TOC 🚀

| toc         | en                      | zh           |
| ----------- | ----------------------- | ------------ |
| algo        | algorithm               | 算法         |
| ai          | artificial intelligence | 人工智能     |
| *appx*      | appendices              | 附录         |
| *attach*    | attachments             | 附件     |
| cc          | compilers               | 编译原理     |
| db          | databases               | 数据库       |
| econ        | economics               | 经济学       |
| hw          | hardware                | 体系架构     |
| langs       | languages               | 编程语言     |
| math        | mathematics             | 数学         |
| net         | networking              | 网络通信     |
| os          | operating system        | 操作系统     |
| robo        | robotics system         | 机器人       |
| sec         | security                | 网络安全     |
| tools       | dev tools               | 开发工具     |
| vision      | visual computing        | 可视计算     |


## License

如未明确说明，原创内容一律使用 CC-BY 4.0 许可。您可以自由使用，但需要保留署名。

## Workflow

笔记库采用 Typst + Markdown 混合排版，推荐的编辑器为 VSCode。
* Markdown 文档的推荐样式见 `appx/theme.css` ，仅供 VSCode Preview 使用。
* Typst 文档的推荐样式见 `appx/theme.typ` 
* 引用信息罗列在 `appx/library.bib`

文件内属性 (YAML Preamble):

| properties |                                                                 | type |
| ---------- | --------------------------------------------------------------- | ---- |
| source     | 来源统一资源定位符，包括网址和源码路径                          | 列表 |
| revised    | 最后大幅修改日期                                                | 日期 |
| copyright  | 作者与出版日期                                                  | 列表 |
| license    | 文内有非 CC-BY 许可证授权下的内容引用时，单个文件许可证可能变化 | 文本 |
| code       | 指文内内容在 `/src` 目录下有对应的源码                          | 列表 |

## Guidelines

- 语言简洁、严谨、准确、逻辑清晰。尽最大努力保持简洁。
- *斜体*的使用：新名词，需要强调的概念等。
- **粗体**的使用：重点强调，突出逻辑。
- 标题级别从二级开始，避免出现四级以下标题，保持层级扁平和简洁。
- 文章篇幅不宜过长，5 个以内三级标题为宜。
- 为了排版美观，不再要求中文强制使用英文标点。
- 中文（全角）和任意半角符号间应隔一个半角空格，括号内侧除外。

### Markdown

- 使用严格断行
- 使用严格 `[]()` 链接与图片引用格式。
- 允许的 Markdown 扩展语法：HTML、Table、Mathjax、Footnote、Mermaid、YAML Preamble 
- 全角标点符号应位于 Markdown 标记之外，如 **粗体之后再逗号**，避免渲染异常。

### Naming

- 普通目录及文件：用 `-` 作为连字符，*尽可能减少空格，全小写* 
- 源码目录及文件：遵循编程语言规范，通常使用 `_` 作为连字符
- 无空格

### Coding

详见 [个人代码风格描述](appx/personal-coding-styles.md)

### Reference

- 仓库整体使用 [CC-BY](license.md) 许可证，部分摘录文章或原创文章使用不同版权许可的，在文章开头标明
- 提及人名或组织名时，使用：@google、@github/jay-waves、@yayvyn 
- 引用时，使用简写格式： `[ITU-T x.800, 2002, p22]` `[Bjarne Stroustrup, 2014]` ，人名、文章名、期刊名皆可以简写。

### Math

- 重要的*定理、引理、定义*使用标题格式，并用数字编号 `x.m.n` 便于整理和引用。
- 数学证明中，用 `\Proof{}` 和 `\square` 标识开始与结束。
- 复杂的数学排版，请优先使用 Typst 而不是 Markdown

### Attachments 

* 优先选择 ASCII/Mermaid 图，非必要不引入图像文件。
* Markdown 优先将图片放在 [OSS](oss.jay-waves.cn) 中，远程引用。支持  `avif, png, webp` 格式。
* Typst 优先选择 `webp` 格式的本地图片，放在 `./attach` 中。
* 图片如有版权信息，同样需要在 `[]` 中著名来源

