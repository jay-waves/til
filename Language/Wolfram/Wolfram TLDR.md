wolfram 有很详细的文档, 见 [wolfram language](https://reference.wolfram.com/language/), 语言编程介绍见 [wolfram fast introduction for programmers](https://www.wolfram.com/language/fast-introduction-for-programmers/en/iterators/)

- `a=1;` 有分号时保留结果, 用于同时输入多个表达式.
- `a=1` 无分号时, 直接输出结果.
- 函数式语言, `shift+enter` 执行后, 语句才会生效.
- wolfram 没有类型, 万物皆符号.

### 运算符

```wolfram
* + / ^ !
(* 这是注释 *)
&& || !
% (* 不是取余, 而是代指上次运算结果, 类似 $? *)
```

## 变量

```wolfram
{x, y} = {1, 2} (*批量赋值*)

{3, 4, 5, a, b, {c,d}, any_object_of_mma} (* 列表类型 *)

in:= Table[i^2, {i, 0, 4}]
out= {0, 1, 4, 9, 16}

{a, b, c}[[2]] (* 取址: b *)

{x, -1, 10, 2}  (* 迭代器类型, min->max->step *)
```

模式匹配:
```wolfram
f=2x  (* 纯符号计算 *)
f/.x->2 (* 4 *)

f=2xy
f/.{x->1,y->2} (* 4 *)

Cases[{f[1], g[2], f[1,2]}, f[_]]  (* {f[1]} *)
Cases[{f[1], g[2], f[1,2]}, f[__]] (* {f[1,2], f[1]} *)
Cases[]
```

变量删除: wolfram 公用一个全局环境, 需要用内置函数手动删除定义, 避免重复.
```wolfram
Clear[f]
```

字符串:
```wolfram
in:= "this" <> "is a" <> "string"
out= "this is a string"

(* 字符串常用于构建哈希表 *)
in:= <|"a"->x, "b"->{5,6}|>
in:= %["a"]
out=x

in:= <|"a"->x, "b"->{5,6}|>[["b",1]]
out= 5

in:= {#b}&[%]
out= {{5,6}}
```

### 数值

wolfram 总是尝试表达最精确的结果, 
```wolfram
in:= 3/7+2/11
out= 47/77  (* 并不计算小数, 防止精度丢失 *)

in:= N[3/7+2/11, 3] (* 用内置函数获取数值解 *)
out= 0.610
```

数值类型转换:
```wolfram
N[x, n]  (* 将 x 转化为实数, n为精度位数 *)
Rationalize[x, dx] (* 将 x 转化为有理数, 误差小于 dx *)
NumberForm[x, n]  (* 将 x 以 n 位精度输出 *)
ScientificForm[x] (* 将 x 以科学计数法形式输出*)
```

### 数值常量

```wolfram
I (* a*i+b *)
Pi
E
Infinity
```

## 函数

wolfram 中内置函数用首字母大写命名, 自定函数可用驼峰法命名.
```wolfram
f[x_, y_, z_]=expr        (* 自定义函数 *)
f[x_]:=expr               (* 延迟定义 *) 
```

使用 `/;` 来定义条件
```wolfram
f[x_,y_]:=x-y/;x>y  (* 当 x>y 时, 函数执行*)

(* If 也是一个内置函数 *)
If[7>5 && 2<=4, a, b] (* a *)
```

匿名函数 (lambda function, pure function): `&`
```wolfram
(#+1)&       (* x+1 匿名函数, # 代表第一个参数*)
{#2, 1+#1, #1+#2}&[a, b]   (* {b, 1+a, a+b} *)

f/@{a,b,c}   (* {f[a], f[b], f[c]} 对多个对象使用函数 *)
```

所有特殊符号都有对应的正式内置函数, 如 `/@` 对应 `Map[]`

### 内置数值函数

wolfram 提供了大量内置函数:
```wolfram
Sin[] Cos[] Tan[] ArcTan[] 
Log[] Log10[] Exp[]
Re[] Im[] Abs[]
Round[] Floor[] Max[], Min[]
Power[] ^, Plus[] +, Times[] *
Fibonacci[] Gamma[] (* 内置特殊函数 *)
```


















