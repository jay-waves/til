## 追加写入 json 文件

有一个 `test.json` 文件，包含内容如下：

```
{
    "key_1": "value_1"
}
```

现需要追加写入 json 文件，向其中增加值，使其包含内容如下：

```
{
    "key_1": "value_1",
    "key_2": "value_2"
}
```

Python 代码实现：

```
import json

new_data = {"key_2": "value_2"}

with open("test.json", "r", encoding="utf-8") as f:
    old_data = json.load(f)
    old_data.update(new_data)
with open("test.json", "w", encoding="utf-8") as f:
    json.dump(old_data, f)
```

如果 `test.json` 文件里面的初始内容为空，则需要在调用 `json.load()` 之前做一个判断，如果内容为空，则需要先构建一个字典，否则会报错：`json.decoder.JSONDecodeError`。

```
import json

new_data = {"key_2": "value_2"}

with open("test.json", "r", encoding="utf-8") as f:
    file = f.read()
    if len(file) > 0:
        old_data = json.load(f)
    else:
        old_data = {}
    old_data.update(new_data)
with open("test.json", "w", encoding="utf-8") as f:
    json.dump(old_data, f)
```

## 更改 json 文件中的值

有一个 `test.json` 文件，包含内容如下：

```
{
    "key_1": "value_1",
    "key_2": "value_2"
}
```

现需要更改 `key_2` 的值为 `value_3`，更改后 `test.json` 文件的内容如下：

```
{
    "key_1": "value_1",
    "key_2": "value_3"
}
```

Python 代码实现：

```
import json

with open("test.json", "r", encoding="utf-8") as f:
    old_data = json.load(f)
    old_data["key_2"] = "value_3"
with open("test.json", "w", encoding="utf-8") as f:
    json.dump(old_data, f)
```

## 用到的方法

-   `json.load()`：将已编码的 JSON 字符串解码为 Python 对象；
    
-   `json.dump()`：将 Python 对象编码成 JSON 字符串；
    
-   `dict.update()`：Python 中把一个字典的键/值对更新到另一个字典里。