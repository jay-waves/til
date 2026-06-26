## 基础字段类型

| 数据类型                     | 含  义                                                         |
| ---------------------------- | -------------------------------------------------------------- |
| CHAR(n) <br> CHARACTER(n)    | 定长字符串 `'my char'`                                |
| VARCHAR(n)                   | 最大长度为 N 的变长字符串, 指定编码存储 (UTF-8, UTF-16LE)                                        |
| CLOB                         | 字符串大对象 (character large object)                          |
| BLOB                         | 二进制大对象(binary large object)                              |
| INT，INTEGER                 | 整数 (4字节)                                                   |
| SMALLINT                     | 短整数 (2字节)                                                 |
| BIGINT                       | 长整数 (8字节)                                                 |
| NUMERIC(p，d)                | 定点数, 由p位数字 (不包括符号, 小数点) 组成, 小数后面有d位数字 |
| DECIMAL(p, d) <br> DEC(p, d) | 同 NUMERIC                                                     |
| REAL                         | 取决于机器精度的单精度浮点数                                   |
| DOUBLE PRECISION             | 取决于机器精度的双精度浮点数                                   |
| FLOAT(n)                     | 可选精度的浮点数, 精度至少为n位数字                            |
| BOOLEAN                      | 逻辑布尔量                                                     |
| DATE                         | 日期, 包含年月日, 格式为 `YYYY-MM-DD`                          |
| TIME                         | 时间, 包含时分秒，格式为 `HH:MM:SS`                            |
| TIMESTAMP                    | 时间戳类型 `'1970-01-01 00:00:01' UTC`                         |
| INTERVAL                     | 时间间隔类型                                                   |
| JSON                             |           结构化数据支持, 概念常见于 [NoSQL](../redis.md)                                                      |

`CHAR(n)` 和 `VARCHAR(n)` 的区别？
* `CHAR(n)` 固定存储 n 长字符。索引更快，但浪费空间
* `VARCHAR(n)` 存储实际字符 + 长度信息。节省空间，不过一般不明显。
* `n` 是指字符长度，不是字节长度。具体字节长度取决于配置的编码。

布尔值：使用 `TINYINT(1)`

## 字段处理函数

字段（Field，列），数据库原始列只有基础数据类型，服务器在查询时需要按需进行映射和处理。

### 字段拼接

mysql 使用 `concat`, 而部分 dbms 则使用 `+` 来拼接.

```sql
select Concat(Rtrim(name), ' (', rank, ') ') as info from students order by name;
```

注, `rtrim` 删除字符串右侧空格. 还有 `trim` 和 `ltrim`. `as` 则指明了新列的别名, 方便后续引用.


### 计算字段

```sql
select name, (math+chinese+english)/3 as avg from students order by name;

# select 可以进行简易测试
select 3*2, Now()
```

### 数据处理函数

数据处理函数, 在不同 DBMS 上实现不同, 可移植性不强; 但编写效率会更高.

#### 文本处理

- `Trim` 去掉左右空格
- `Left` 返回串左边字符, `Right` ...
- `Locate` 找出串的一个子串
- `Length` 返回串长度
- `Lower` 转换为小写, `Upper` 转为大写.

#### 汇总统计

- `Avg`: 平均值, 忽略 null
- `Count`: `Count(column)`, 统计具体行, 忽略null; `Count(*)`, 统计表中行数, 包括 null.
- `Max`, `Min`: 忽略 null
- `Sum`: 忽略 null

```sql
select Avg(math) as math_avg from students;
```

#### 日期处理函数

mysql 的日期格式首选为 `yyyy-mm-dd`

```sql
select name from students where Date(birthday) = '1999-09-12';
# Date 可以删掉 birthday 字段的时间值
```

- `AddDate` 增加一个日期 (天, 周)
- `AddTime` 增加一个时间 (时, 分)
- `CurDate` 返回当前日期
- `CurTime` 返回当前时间
- `Date` 返回日期时间的日期部分
- `DateDiff` 计算两个日期之差
- `Date_Add` 高度灵活的日期运算函数
- `DateFormat` 返回一个格式化的日期或时间串
- `Day` 返回一个日期的天数部分
- `DayOfWeek` 对于一个日期 ， 返回对应的星期几
- `Hour` 返回一个时间的小时部分
- `Minute` 返回一个时间的分钟部分
- `Month` 返回一个日期的月份部分
- `Now` 返回当前日期和时间
- `Second` 返回一个时间的秒部分
- `Time` 返回一个日期时间的时间部分
- `Year` 返回一个日期的年份部分

#### 数值函数

数值函数各个 DBMS 比较统一, 当然, 使用也不频繁.

- `Abs` 返回一个数的绝对值
- `Exp` 返回一个数的指数值
- `Sqrt` 返回一个数的平方根
- `Mod` 返回除操作的余数
- `Pi` 返回圆周率
- `Rand` 返回一个随机数
- `Sin` 返回一个角度的正弦
- `Cos` 返回一个角度的余弦
- `Tan` 返回一个角度的正切

