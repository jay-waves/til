![](../attach/gartner_hype_cycle.avif)


## 标准库

标准库定义了一系列开箱即用的模块和工具, 加快开发效率, 屏蔽跨平台的复杂性, 关注的内容有:

- 数据结构与算法: 数组, 列表, 字典, 哈希表, 栈, 树等.
- 字符串: 字符串处理, 编码转换, 正则表达式等.
- IO: 文件读写 (非文件系统), 标准输入输出, 底层网络通信 (套接字)
- 基础数值计算: 浮点数与整数算术, 随机数, 复数
- 线程并发, 同步机制 (锁, 信号量, 原子操作等). 
- 内存管理: 内存垃圾处理, 内存机制
- 编程范式支持: 面向对象, 函数式
- 错误处理及调试分析工具: 异常, 断言, 错误码, 调试, 性能分析, 单元测试
- 包管理: 模块化, 构建工具, 文档生成, 代码格式化.
- 其他开发工具: 序列化 (json, blob, xml), 解压缩, 日志, 密码, 配置管理 (toml, yaml, json, ini)

### 写在别处

- 操作系统相关服务, 见 [System/Programming](../System/Development/ReadMe.md), 包含:
	- 日期与时间, 本地化
	- 文件系统: 文件夹管理和遍历等, 文件元信息处理. 
	- 进程管理: 主要是 POSIX 系列系统调用, 以及进程间通信.
	- 编码转换, 序列化, 解压缩 (先放在这)
- 密码库, 见 [Security/密码学](../网络安全/密码学/README.md)
- 网络与事件异步编程, 见 [Network/Programming](../操作系统/io/ReadMe.md)

注意. 和功能强相关的部分, 不区分具体语言.

## 包管理

包管理代表着语言生态以及易用程度. 通过包管理工具, 自动安装项目需要的依赖或第三方库, 避免手动管理的繁琐. 常见内容有:

- 包仓库: PyPI, npm
- 包管理工具: 下载, 安装, 更新, 删除, 编译, 创建发布
- 依赖管理, 版本管理, 包配置文件
- 虚拟环境


> -- Jeff Atwood 
> 
> Code Tells You How, Comments Tell You Why.

## 名词表

### 1. 编程范式

#### TagDispatch

一种函数反射方法, 见 [运行时函数多态](cpp/面向对象/运行时函数多态.md#函数体).

#### System Call

系统调用. 见 [中断与异常](../操作系统/进程调度/中断与异常.md).

#### Reflection

反射, 在**运行时**操作和配置对象及其属性, 在编写代码时这些类和方法可能不是已知的, 常用于实现框架和库. 如 Python 的 [python 元编程](python/python%20元编程.md).

### Poll

轮询结构. 并发程序模型中, 某个线程阻塞等待其他资源, 线程将不时主动查询资源是否可用来结束轮询状态.

##### ORM

Object-Relational Mapping, 软件开发概念, 将面向对象的类资源/方法和一个关系型数据库绑定起来, 对类操作即等价于对数据表操作. 类的元类等价于数据库表, 一个类实例等价于表中的一行.

#### OO

object-orientation, 面向对象编程.

#### Lazy Binding

延迟绑定.

#### Generics

泛型, 即允许接口同时复用于多种数据类型, 使用类型参数化实现, 在编译时确定具体数据类型 (即编译器为不同数据类型的调用生成不同代码, 而不是真的动态类型). 如 CPP 模板.

#### Function Signature

函数签名. 见 [compiler/linking/符号](../编译原理/链接过程/符号.md)

#### Casting/Coercion

显式类型转换/隐式类型转换

### 2. 并发相关

#### Semaphore

信号量. 见 [进程同步与互斥](../操作系统/进程调度/进程同步与互斥.md).

#### Synchronization

进程间同步, 见 [进程同步与互斥](../操作系统/进程调度/进程同步与互斥.md).

#### Read-Write Lock

读写锁. 见 [进程同步与互斥](../操作系统/进程调度/进程同步与互斥.md)

#### Reentrant

可重入.

#### Preemption

抢占式调度. 见 [进程调度](../操作系统/进程调度/进程调度.md)

#### Interrupt

操作系统中断, 见 [中断与异常](../操作系统/进程调度/中断与异常.md)

#### ISR

interrupt service routine, 中断处理程序.

#### Mutex

多线程互斥量. 见 [进程与线程](../操作系统/进程调度/进程与线程.md)

#### C10K

如何让一台服务器同时处理 10K 个并发连接, 同时保持高性能.

> 1999. Dan Kegel. The C10K Problem.

### 3. 软件工程

### IaaS

Infrastructure as a Service, 一种云服务交付模型.

### SaaS

Software as a Service, 一种云服务交付模型.

#### SDK

software development kit, 软件开发工具集.

#### EOL

End of Life, 软件或硬件停止维护的状态.

#### DOM

文档对象模型, 指网页文档的编程接口, 允许脚本语言 (JS) 修改网页内容结构.

#### Brain-damaged, Brain-dead

错误和傻逼的产品设计, 无法使用和接受, 对大脑产生损害.

#### Boilerplate 

"样板文件", "陈词滥调". 指无需修改, 可以重复使用的固定陈述.

#### CI/CD 

持续部署, 持续集成? donnot push your production on friday.

#### Silver Bullet

银弹. 西方传说中只有银弹才能杀死狼人, 巨人和巫师. 人们把在软件体系结构中添加抽象层以解决兼容性问题的做法也叫做 " 银弹 ", 用以形容其是能解决各种问题的万灵药.

#### API

application programming interface, 应用程序编程接口

#### ABI

application binary interface, 应用程序二进制接口.