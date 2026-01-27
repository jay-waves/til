---
revised: 25-01-24 
---

> -- Robert Kowalski. **Algotirhm = Logic + Conrtol**. 1979
> 
> An algorithm can be regarded as consisting of a logic component, which specifies the
knowledge to be used in solving problems, and a control component, which determines
the problem-solving strategies by means of which that knowledge is used. The logic
component determines the meaning of the algorithm whereas the control component
only affects its efficiency. The efficiency of an algorithm can often be improved by
improving the control component without changing the logic of the algorithm. We argue
that computer programs would be more often correct and more easily improved and
modified if their logic and control aspects were identified and separated in the program
text.

> -- Niklaus Emil Wirth. 1976.
>
> **Algorithms + Data Structures = Programs**

通过泛型来标准化*数据结构*, 从而标准化*控制*, 从而适应任意的*业务逻辑*. 代码的实际复杂度是业务逻辑, 其他两者应与其解耦.

## 编程范式

|          | C                       | C++              | Go         | Java | Python | JavaScript | Rust              | Haskell |
| -------- | ----------------------- | ---------------- | ---------- | ---- | ------ | ---------- | ----------------- | ------- |
| 类型声明 | 静态                    | 静态             | 静态       | 静态 | 动态   | 动态       | 静态              |         |
| 类型转换 | 弱类型                  |                  |            |      |        |            |                   |         |
| 算法泛型 | 宏, `void*`             | 类多态, 函数重载 |            |      |        |            |                   |         |
| 类型泛型 | `void*`                 | 模板,            |            |      |        |            |                   |         |
| 范式     | 过程式                  |                  | 过程式     |      |        |            |                   | 函数式  |
| 面向对象 |                         | 类               | 结构体方法 | 类   | 类     | 基于原型   |                   |         |
| 资源管理 | 手动管理堆 + 局部作用域 | RAII             |            |      |        |            | 生命周期 + 所有权 |         |

图灵完备的编程语言中, 编程范式大概分为:
- 命令式 (Impreative Programming)
	- 过程式 (Procedural), C 
	- 命令式, Matlab
- 声明式 (Declarative Programming)
	- 函数式, Hashkell 
	- 逻辑式, Prolog
- 面向对象, Java 

![|800](../../attach/Pasted%20image%2020250507215501.avif)

### 面向对象

> -- Design Patterns
> 
> Program to an 'Interface', not an 'Implementation'
> 
> Favor 'Object Composition' over "Class Inheritance"

原型编程: 不要求子对象有相似的内存结构, 方法和数据皆可以修改 (slots)

## 重构

* 重构前, 确保有一套完整的测试集. 
* 好代码的检验标准是: 能否让别人能够维护.
* 重构时, 每一步修改都要小, 修改后提交 GIT 并运行 CI/CD.
* 没人能预先做出良好的设计, 重构必不可少.

### 代码的坏味道

* 神秘的命名
* 重复的代码
* 过长的函数, 以及过长的参数列表
* 使用全局数据
* 使用可变数据和结构
* 过度追求通用性
* 过长的消息及委托链条
* 过大的类
* 注释: 代码不能说明自己

### 测试

* 测试应和代码写在一起??
* 每当收到一个 BUG, 应写一个单元测试来暴露这个 BUG.

### 重构方法

* 提炼函数, 并花心思起名字
* 封装数据, 限制其可见范围. 优先使用常量.
* 封装参数对象
* 用查询取代临时变量
* 拆分循环: 确保每个循环只做一件事, 即使代价是性能.
* 用函数式编程取代循环.
* 分解条件表达式
* 减少条件语句嵌套
* 减少函数控制流对参数的依赖, 如果参数是 Flag, 更好的方法是拆分为两个明确的函数.

## Reference

https://en.wikipedia.org/wiki/Software_design_pattern 

编程范式游记. 左耳朵耗子. 

重构. Martin Fowler. 2020