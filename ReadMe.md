### TOC 🚀

| 英文标题      | 中文标题 | 描述                                               |
| ------------- | -------- | -------------------------------------------------- |
| Algorithm     | 算法     | 数据结构, 算法                                     |
| Intelligence  | 人工智能 |                                                    |
| Compiler      | 编译     | LLVM, 链接过程, 汇编                               |
| Information   | 信息     | 数据库, 信息论, 数字信号     |
| HardWare      | 硬件     | 计算机组成                                                   |
| Language      | 编程语言 | C, C++, Go, Python, Wolfram, 设计模式                        |
| Network       | 网络     | 网络分层模型, 防火墙, 网络代理                     |
| System        | 操作系统 | Linux, Windows, Shell                   |
| Security      | 网络安全 | 网络安全, 软件安全, 密码学                   |
| Math          | 数学     | 数学就是数学 |
| Tools         | 工具     | Git, Vim, Latex, IDE, ...                     |
| attach        | 图片附件 |  去透明通道压缩后图片, 无第三方版权                                                  |
| appx          | 附录     | 技术标准, 参考信息, 对照表, 杂项                               |
| paper         | 论文     |                                                    |
| src           | 代码     |                                                    |

### License

如未明确说明, 原创内容一律使用 CC-BY 4.0 许可. 您可以自由使用, 但需要保留署名.

### Workflow

分为内容标签和进度标签两类, 进度标签不会出现在正文中.

| tags                      |                      |
| ------------------------- | --------------------------- |
| #LGTM #KeyPoints          | 有兴趣, 重点      |
| #Trouble #TroubleShooting | 遇到的难题, 故障处理经验        |
| #Doubtful    | 存疑  |
| #FAQ                      | 提问, 一些好奇心            |
| #NSFW                     | 奇怪内容, 加入 `.gitignore` |
| #Code                     | 笔记内容为大段源码分析              |
| #Docs                     | 笔记内容为复述文档或定义    |
| #Repost                   | 笔记内容为转载              |
| #Paper                                  | 笔记内容为文献. 文末会列出来源及文献类型.                    |
| #Recipes                  | 最佳实践                    |
| #ToDo                     | 进度标签, 未来会投入精力    |
| #NoPlan                   | 进度标签, 未来不会投入精力  |
| #WIP                      | 进度标签, 正在              |
| #Recall                   | 进度标签, 需要回忆复习      |
| #TypeSet                  | 进度标签, 需要调整排版      |

文件内属性:

| properties |                                                                 | type |
| ---------- | --------------------------------------------------------------- | ---- |
| source     | 来源统一资源定位符, 包括网址和源码路径                          | 列表 |
| revised    | 最后大幅修改日期                                                | 日期 |
| copyright  | 作者与出版日期                                                  | 列表 |
| license    | 文内有非 CC-BY 许可证授权下的内容引用时, 单个文件许可证可能变化 | 文本 |
| code       | 指文内内容在 `/src` 目录下有对应的源码                          | 列表 |
| tags       | 内容标签, 见上                                                  | 列表     |


### Writing Guidelines

- 全部**使用英文标点**, 半角标点前或后添加一个半角空格.
- 中文 (全角) 和任意半角符号间应隔一个半角空格, 括号内侧除外.
- 文件名可以包含空格. 英文大小写皆支持.
- 标题级别从二级开始, 最低四级标题.
- 标题中可以有空格, 编号和题目间应有空格, 次级编号用 `.` 分隔.
- 文章篇幅不宜过长, 5个以内三级标题为宜.

- 语言简洁有力, 减少口语和形容词, 减少重复叙述, 体现清晰发展逻辑.
- 强调句使用粗体 `**`, 新名词使用斜体 `*`, 名词收录到[名词表](Glossary.md)中.
- 推荐字体使用 "文泉驿等宽微米黑" 或 "等距更纱黑体 SC", 代码段字体使用 Fira Code.
- 推荐使用 Obsidian 渲染器的 Github Theme 主题.
- Markdown 开启严格断行, 关闭 wikilink 格式, 尽量使用标准语法.

#### Coding Style

- 嵌入 C 代码使用 [Kernel 风格](Language/C/Kernel%20C%20Style.md).
- 嵌入 C++ 和 Go 代码使用 [Google 风格](Language/C++/Google%20C++%20Style.md).
- 嵌入 Python 使用 [PEP8 Style](Language/Python/PEP8%20Style.md) 风格.
- 不要求控制行长度, 建议代码行长度不超过 100 字符, 全角占两字符.
- `/src` 文件夹中, 默认使用原文件格式, 如 C 语言使用 `.c` 文件; 
- `/src` 文件夹中, 需要记录笔记的, 使用类 Jupter 格式, 如 Python Notebook 使用 `.py.md` 文件, 其中 `.py` 仅起到标注语言类型作用.

#### Reference Style

- 大篇幅参考时, 在开头文章元信息处列明来源. 
- 小篇幅参考时, 在文内用脚注列明来源. 脚注内容应统一放在文末, 不要穿插在文间.
- 图片应在 `[]` 括号中列明来源, 尽量自行绘制, 并压缩空间.
- 仓库整体使用 [CC-BY](License.md) 许可证, 部分摘录文章或原创文章使用不同版权许可的, 在文章开头标明.
- 文内引用学术论文时, 括号中使用 Harvard 风格注明作者和发表年份: `[yjw, 2023]` 或 `(yjw, 2023)`, (可选) 链接到论文 URI. 
- 文内引用国际标准时, 括号中使用 Harvard 风格注明标准名称和发表年份: `[ITU-T X.800, 1111]` 或 `(RFC 3556, 2023)`.
- 文内引用著作时, 括号中使用 IEEE 风格, 即 Markdown 注脚语法 `[^1]`, 并标明著作名和页数.

#### Math Style

- 数学中独立的*定理, 引理, 定义, 例子*皆使用标题格式, 便于链接和折叠. *证明, 命题*使用正文加粗格式.
- 数学标题应使用数字层次标号, 如 `1.3.1`.
- 数学定理和定义内容使用粗体, 用 `<br>` 增长段间距.
- 数学证明中, 每行 (段内换行) 要不是完整的因果关系 "因为..所以..", 要不是上一行的直接推论 "于是.." "从而.." "那么..". 
- 数学证明中, 用 *证明:* 和 $\blacksquare$ 标识开始与结束.
- 数学公式符号 `$$` 和左右文字间隔一个半角空格, 避免渲染问题.

> 免责:   
> 标准在不断迭代, 部分笔记可能不符合上述标准, 也不会修改.   
> 包含部分 Obsidian 扩展 Markdown 语法. 可能不兼容 [GFM 语法](https://github.github.com/gfm/).
