常见编码与格式:

| 名称   | 作用                                                                | 例子                                                         |
| ------ | ------------------------------------------------------------------- | ------------------------------------------------------------ |
| [Base64](../../Information/Encoding/Base64.md) | 用于将文件/图片的二进制数据, 编码为纯 ASCII 字符串.| `YGlklD4gZXh6ZHYudHh0YA==`                                                          |
| [Base58](../../Information/Encoding/Base58.md) | 类似 Base64, 去除了易混淆的 O0Il, 用于比特币                        |                                                              |
| Hex    | 二进制转化为十六进制字符串. 用于加密和校验                          |                                                              |
|  [XML](../../Information/Data%20Exchange/XML.md)   |                                                                     |                                                              |
| [YAML](../../Information/Data%20Exchange/YAML.md)   | 类似 JSON, 更反人类一点. 既不易读, 也不易懂...                                                                    |                                                              |
| [JSON](../../Information/Data%20Exchange/JSON.md)   | 文本交换格式, 自带简单数据结构, 便于解析.                             |                                                              |
| HTML   | 用于在 HTML 中标识特殊字符, 避免和标签关键字冲突.                   | `&apos, &lt, &gt, &times, &divide, &amp, &quot, &nbsp, &copy, &reg` |
| [URL Encoding](../../Information/Encoding/URL%20Encoding.md)   | 用于避免 URL 中的特殊字符                            | `Hello%20world%21`                                           |

## 不同文件格式

### `pandoc`

[`pandoc`](http://pandoc.org/) 用于 Mardown, HTML 和其他任意格式的转化. 

> 我愿称之为 21 世纪最伟大软件. 

### `jq`, `shyaml`, `xmlstarlet`

[`jq`](http://stedolan.github.io/jq/) 处理 Json. 算是上古神器了, 衍生出 `fq, jq, fx` 等一大批子孙后代.

```sh
jq .[0]
jq .question
```

### `shyaml`

[`shyaml`](https://github.com/0k/shyaml) 处理 YAML

### `xmlstarlet`

`xmmlstarlet` 处理 XML.

### `csvkit`

[`csvkit`](https://github.com/onyxfish/csvkit) 用于处理 Excel 或 CSV 文件, 提供了 `in2csv`，`csvcut`，`csvjoin`，`csvgrep` 等工具

## 二进制数据处理

### `hd, hexdump, xxd`

以十六进制显示二进制文件.

### `bvi, hexedit, biew`

编辑二进制文件.

### `strings`

从二进制中抽取文本或特定比特, 用于逆向和调试.

```bash
# 提取长度至少为6的字符串, 并输出其字节偏移量(以十六进制)
strings -tx -n 6 --encoding={s,S,b,l,B,L} example.bin
```

## 编码转换

### `iconv`, `uconv`

`iconv` 更改[Char Encoding](../../Information/Encoding/Char%20Encoding.md)方式. 

```sh
# 转换为 Windows 默认的 UTF-16LE 编码 
iconv -f utf-8 -t utf-16le data.txt

iconv -f GBK -t UTF-8 input.txt -o output.txt
```

`uconv` 则支持一些高级 Unicode 功能, 在 ICU 工具包中.

### `expand`, `unexpand`

制表符与空格间转换.

### `md5sum`, `sha256sum`, `openssl`

```sh
openssl dgst -md5 
```

### `base64`

### `urlencode`

### `python`

与其寻找零碎的命令行软件, 不要忘了 python 提供了相当强大的标准库.

```python
# base64
import base64

base64.b64encode()
base64.b64decode()

# ASCII
ord(char)
chr(num)

# Hex
my_str = "..."
hx = my_str.encode().hex()
decoded_hx = bytes.fromhex(hx)

# URL
import urllib.parse
urllib.parse.quote(original_url)
urllib.parse.unquote(encoded_url)

# HTML
import html
text = '<div>Hello &</dev>'
encoded_text = html.escape(text) # &lt;div&gt ...
decoded_text = html.unescape(encoded_text)
```

> 这里推荐另一个软件: CyberChef, 处理流信息的瑞士军刀.