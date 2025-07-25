- 1960-1980 早期编程语言: Fortran, Lisp (垃圾回收), ALFO --> BCPL --> C --> C++, Basic --> Smalltalk (面向对象) --> Object C Rust
- 1980-2000 跨平台的, 自动内存管理, 动态类型: Python, Perl --> Ruby, Java, C#, Lua, JavaScript 
- 2000-2010 异步和并发, 内存和类型安全: Rust, ObjectC --> Swift, Java --> Kotlin, Golang, JavaScript --> TypeScript 


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
- 密码库, 见 [Security/密码学](../Security/密码学/README.md)
- 网络与事件异步编程, 见 [Network/Programming](../Network/Development/ReadMe.md)

> 和功能强相关的部分, 不区分具体语言.

## 包管理

包管理代表着语言生态以及易用程度. 通过包管理工具, 自动安装项目需要的依赖或第三方库, 避免手动管理的繁琐. 常见内容有:

- 包仓库: PyPI, npm
- 包管理工具: 下载, 安装, 更新, 删除, 编译, 创建发布
- 依赖管理, 版本管理, 包配置文件
- 虚拟环境


> -- Jeff Atwood 
> 
> Code Tells You How, Comments Tell You Why.