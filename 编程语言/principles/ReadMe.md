---
revised: 25-01-24 
language: [c++, java]
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

## 设计模式

设计模式 (Design Pattern)

1. 创建型模式:
	- 工厂模式 (Factory Pattern)
	- **抽象工厂模式 (Abstract Factory Pattern, Family Creator)**. 创建一组相关的对象.
	- **单例模式 (Singleton Pattern, One and Only)**. 全局唯一对象.
	- **建造者模式 (Builder Pattern, Lego Master)**. 按步骤创建对象.
	- **原型模式 (Prototype Pattern, Cloner)**. 学习样例创建副本.
2. 结构型模式:
	- **适配器模式 (Adapter Pattern, Universal plug)**. 连接不同的接口.
	- **桥接模式 (Bridge Pattern, Connector)**. Link what it is to how it works.
	- 过滤器模式 (Filter, Criteria Pattern)
	- **组合模式 (Composite Pattern, Tree Builder)**. 创建树形数据结构.
	- **装饰器模式 (Decorator Pattern, Customizer)**. 向已存在对象添加新功能. 
	- **外观模式 (Facade Pattern, One-stop shop)**. 整个系统对外仅暴露简单接口.
	- **享元模式 (Flyweight Pattern, Space Saver)**. 共享小的可复用的资源.
	- **代理模式 (Proxy Pattern, Stand-In Actor)**. 
3. 行为型模式:
	- **责任链模式 (Chain of Responsibility Pattern, Request Relay)**. 不断向下传递请求, 直到负责者.
	- **命令模式 (Command Pattern, TaskWrapper)**, 将请求变为一个就绪对象
	- 解释器模式 (interpreter Pattern)
	- **迭代器模式 (Iterator, Collection Explorer)**.
	- **中介者模式 (Mediator, Communication Hub)**. 用中介来简化对象间的通信.
	- **备忘录模式 (Memento, Time Capsule)**. 捕获并保存对象状态.
	- **观察者模式 (Observer, Broadcaster)**. 将事件同步 (通知) 给他人
	- 状态模式
	- 空对象模式
	- 策略模式
	- 模板模式
	- **访问者模式 (Visitor, Skillful Guest)**. 熟客直接操作对象内部.

### 设计模式原则

1. 开闭原则 (Open Close Principle): 对扩展开放, 对修改关闭, 实现热插拔.
2. 里氏代换原则 (Liskov Substituition Principle, LSP): 任何基类可出现的地方, 子类一定可以出现.
3. 依赖倒转原则 (Dependence Inversion Principle): 面向接口编程, 依赖于抽象而不依赖于具体.
4. 接口隔离原则 (Interface Segregation Principle): 降低耦合度. 使用多个隔离的接口, 优于使用单个巨型接口.
5. 迪米特法则 (Demeter Principle): 少与其他实体发生相互作用, 保持独立性.
6. 合成复用原则 (Composite Reuse Principle): 使用组合, 而不使用继承.

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