---
revised: 25-01-24 
language: [c++, java]
---

Patterns and Paradigms

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

### 面向对象

(Object-Oriented Programming)

- Java
- Python

### 命令式

(Impreative Programming)

- C, Pascal: 过程式 (Procedural)
- Bash, MMA, Fortran: 命令式
- JS, Go: 并发式 (Concurrent)

### 声明式

(Declarative Programming)

- Html, SQL: 声明式
- Hashkell: 函数式
- Prolog: 逻辑式

## Reference

https://en.wikipedia.org/wiki/Software_design_pattern