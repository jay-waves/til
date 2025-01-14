Lua 是一款方便嵌入 C/C++ 代码之中的轻量级脚本语言. 

```bash
lua hello.lua
```

导入其他 lua 脚本:

```lua
dofile('lib1.lua')
```

注释语法:

```lua
-- one line comments

--[[ 
	multi-line long comment
--]]

--[[ 
	print(10)  --> 10, commented
--]]

---[[ 
	print(10)  --> 10, uncommented
--]]
```

栈大小为一百万个实体.

## 类型系统

Lua 是动态类型的语言, 未初始化的变量值一律为 `nil`. `nil` 的语义就是空值, 赋值为 `nil` 的任意变量都将从内存被删除.

Lua 基础类型有:

```lua
type(nil)          --> nil
type(ture)         --> boolean
type(10.4)         --> number
type("Hello")      --> string
type(io.stdin)     --> userdata
type(type)         --> function
type({})           --> table
type(type(...))    --> string

type(a)            --> nil
a = 10
type(a)            --> number
a = "hello"   
type(a)            --> string
```

`userdata` 可用于存储任意 C 程序的数据, 如任意用户定义的结构体. 变量本身没有类型, 跟随其蕴含的值的类型.

### 布尔类型

Lua 提供 `and, or, not` 关键字, 也支持表达式 `<, >, <=, >=, ==, ~=`

| type    | condition test |     |
| ------- | --------- | --- |
| `false` | false     |     |
| `nil`   | false     |     |
| `0`     | true      |     |
| `""`    | true          | empty string    |

### 数字类型

Lua 只有两种数字类型: 64 位整数, 和双精度浮点数. 当然 Lua 也可以被编译在 32 位平台上 (称为 Small Lua), 使用 32 位整数和单精度浮点数. 整数直到 Lua5.3 才被正式引入, 之前都用双精度浮点数.

```lua
0.4
4.57e-3
0.3E+20
0x1A3

type(3.0)      --> number
type(3)        --> number

math.type(3.0) --> float 
math.type(3)   --> integer
```

常见数字运算符: 对于除法和指数操作, Lua 强制将结果转化为浮点数.

```lua
- + * /  ^
//   --> floor division
%    --> a % b == a - ((a // b) * b)
```

取余操作常用于获得固定位数的浮点数:

```lua
x = math.pi
x - x % 0.01   --> 3.14
x - x % 0.001  --> 3.141
```

数字类型转化: 由于双精度浮点数的最高实际有效位位数只有 53 位, 所以有效位数更多的整数转化为浮点数时会有精度损失. 这可能会导致 Lua5.3 和之前版本程序的不兼容性.

```lua
3 + 0.0  --> 将整数转化为浮点数
2^53 | 0 --> 将浮点数转化为整数. 注意被转化的浮点数只能是 0~2^53 的整数值, 否则会报错
math.tointeger(2^64) --> nil, 无法不丢失精度地转化时, 不显式报错, 返回 nil
```

常用数学库函数:

```lua
math.huge         --> inf
math.sin(math.pi / 2)
math.max(10.4, 7, -3, 20)

math.random()     --> uniform distribution in [0,1)
math.random(n)    --> integer in [1,n]
math.random(n, m) --> integer in [n,m]
--[[ 
对于随机数函数, Lua 解释器默认使用的种子总为 `1`, 
这意味着每次独立运行程序时, 产生的随机数序列是相同的.
]]
math.randomseed(os.time())

math.floor(3.3)      --> 3
math.floor(-3.3)     --> -4
math.ceil(3.3)       --> 4
math.ceil(-3.3)      --> -3
math.modf(3.3)       --> 3   0.3
math.modf(-3.3)      --> -3  -0.3

--[[ 
浮点数形式表示的整数取整时, 可能因精度误差导致错误.
]]
function round(x)
	local f = math.floor(x)
	if (x == f) or (x % 2.0 == 0.5) then 
		return f --> for 2.5 or 2, return 2
	else
		return math.floor(x + 0.5) --> nearest integer to x
	end
end
```

## 字符串

在 Lua 中, 字符串是不可变类型, 由任意大小字节串表示. 字符串可用单引号, 也可用双引号, 示字符串内容灵活选择. Lua5.3 引入了更广泛的 UTF-8 支持.

```lua
a = "hello"
b = "hello\nworld"
c = 'hello\nworld'
d = '\049\x41\u{3b1}' --> \ddd \xhh 
e = '\u{3b1}'         --> \u{hhh..} 表示 UTF-8 字符, Lua5.3 引入
f = [[ 
	<html>
	<head>
		..... 长字符串
	</head>
	...
]]
g = [===[
	如果字符串中包含 [[]], 可以在中括号之间添加任意数量的等号
]===]

```

常用字符串库函数: 由于字符串是不可变类型, 所以库函数通常返回一份修改后的副本 (一个新字符串).

```lua
a = "hello"
#a                    --> 获取字节长度, 取决于具体的字符串编码方式.
string.len(a)
"hello " .. "world"   --> 链接操作.

tonumber("100101", 2) --> 37
tostring(10)          --> "10"

string.rep("a", 3)    --> "aaa"
string.reverse("abc") --> "cba"
string.lower(s)
string.lower(s)
string.sub(s, i, j)   --> 提取字符串的一部分 s[i,j]

string.char(97)       --> "a"
string.byte("a")      --> 97

string.format("%s %d %x", 10, 200, "a") --> "10 c8 a"
string.format("%.4f %02d", math.pi, 1)  --> "3.1416 01"

-- 库函数也可以作为对象的"方法"被使用
s:upper()

-- UTF-8 标准库, 在字面值语义上, 而不是在 Unicode 层面操作字符串.
utf8.len("你好")           --> 2
utf8.len("ab\x93")         --> nil  3  非法的 UTF-8 字节串, 将返回 nil 和出错位置
utf8.char(133, 233, ...)
utf8.codepoint("你好", 2)  --> 类似 stirng.byte(), 但返回值无语义
utf8.offset("你好", 2)     --> 获取 "好" 在实际字节种的偏移
utf8.codes("你好")
	--> 1   65
	--> 2   231

```

## 参考

Programming in Lua. Fourth edition.