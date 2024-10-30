---
tags: [FAQ]
---

## Word

### 如何写伪代码?

1. 创建$n\times 2$ 表格, 将第一列调窄. 
2. 分别合并一二行左右单元格, 然后去除多余边线, 只保留第一行 第二行和最后一行底线.
3. 自动填写编号: 摁`ctrl+F9`插入域, 输入`seq n \# 0`, 即从0开始递增的域. 然后选中整个表格第一列, 批量复制过去, 然后摁F9更新域. 数字会从1开始自动递增.

> PS: 现在有插件了: Easy Code Formatter

### 如何设置多张子图?

0. 打开"表格工具-布局-查看网格线"选项
1. 使用两行表格, 第一行填充图片, 第二行插入小题注. 去除边线
2. 整个表格插入大题注

### 行间公式如何标号?

0. `alt`+`=` 插入行间公式
1. 输入公式后, 在公式最后添加 `#(1)`, 然后回车

### Latex 如何转换为 word/ppt?

使用 pandoc, 实质是先转换为 markdown, 然后转换为 word.
- `pandoc -s main.tex -o main.docx`
- `pandoc -s main.tex -o main.pptx`

### Word 异常卡顿

设置 > 加载项 > 管理 > COM加载项, 禁用全部加载项.

## Excel

### Excel 异常卡顿

1. 设置 > 加载项 > 管理 > Excel/COM加载项, 禁用全部加载项.
2. 设置 > 高级 > 常规 > 请求自动更新链接. 关闭.
3. 设置 > 高级 > 显示 > 禁用硬件图形加速. 关闭. (Excel 默认用的是核显, 可以在 Nvidia 控制面板设置为独显, 效果未知)

## SearchEngine

1. 完全匹配: `"my question"`
2. 特定站点: `site:zhihu.com my question`
3. 查询字典: `define:myword`
4. 排除某个词汇: `-csdn`
5. 特定文件类型: `filetype:pdf`
6. 通配符: `* of money`
7. 逻辑暗示: `and, or`
8. 筛选年份: `after:2020`, `before:2019`, `2010..2020`
9. 近义词: `~mobile phone`
10. 搜索 url 内内容, `inurl:edu`