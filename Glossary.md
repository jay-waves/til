# A

### API

application programming interface, 应用程序编程接口

### ABI

application binary interface, 应用程序二进制接口.

### Assembly

汇编

### Arch

轻量和简约的 Linux 发行版, 滚动更新.

### Address Sanitizer

Google 的 C/C++ 运行时内存错误检测工具, 基于 LLVM IR 实现.

### AES

目前广泛使用的对称加密标准, 见 [AES](Security/密码学/分组密码/SP-结构/AES.md)

### Avalanche Effect

雪崩效应 (avalanche effect) 指现代加密算法的一种理想属性. 当明文输入产生微小变化 (如一比特翻转) 时, 会引起算法输出的大面积无规律改变.

### ACL 

[Access Control List, 访问控制列表](Network/防火墙/Firewall.md), 用于定义哪些用户或系统进程可以访问系统资源以及权限级别.  

### AES-GCM 

AES-[Galois Counter Mode](Security/密码学/分组密码/链接模式.md), 一种结合分组加密和认证的模式, 广泛应用于现代加密协议如 TLS.  

### ARP

[Address Resolution Protocol, 地址解析协议](Network/网络层/ARP.md), 用于将 IP 地址解析为物理地址 (MAC).  

### ASLR

Address Space Layout Randomization, 地址空间布局随机化, 用于防止缓冲区溢出攻击. 

---

# B

### BFD

binary file descriptor library, 二进制文件描述符库.

### Big-endian

大端字节序. 见 [端序](HardWare/端序.md)

### Bus

总线

### Bootstrap

自举

### Silver Bullet

银弹. 西方传说中只有银弹才能杀死狼人, 巨人和巫师. 人们把在软件体系结构中添加抽象层以解决兼容性问题的做法也叫做 " 银弹 ", 用以形容其是能解决各种问题的万灵药.

### BSS

block started by symbol, ELF 文件中存储未初始化全局变量和局部静态变量的段. 见 [Unix-ELF](Compiler/Linking/Unix-ELF.md)

### Brain-damaged, Brain-dead

错误和傻逼的产品设计, 无法使用和接受, 对大脑产生损害.

---

# C

### Clang

基于 LLVM 的 C/C++/Objective-C 编译器前端. 见 [Comiler/LLVM/clang](Compiler/ToolChain/LLVM/clang.md)

### CI/CD

continuous integration/continuous deployment, 用于自动化测试/集成/部署, 来加快软件交互速度.

### Completeness

完备性. 逻辑系统是完备的, 如果它能证明该逻辑下的**所有**真命题. 见 [可靠性与完备性](Math/计算理论/可靠性与完备性.md)

### Casting/Coercion

显式类型转换/隐式类型转换

### COFF

common object file format, ELF 格式前身. 见 [Unix-ELF](Compiler/Linking/Unix-ELF.md)

### COM

component object model, 组件对象模型.

### Complilation

编译. 见 [编译过程](Compiler/编译过程.md)

### CVE

Common Vulnerabilities and Exposures. 用于标识和追踪已知的网络安全漏洞与安全问题, 每个 CVE-ID 对应一个漏洞及其描述/影响范围/补丁信息.

### CWE

Common Weakness Enumeration, 常见弱点枚举. 用于分类和描述各种软件安全中的弱点, 比如 CWE-89 指的是 SQL 注入弱点.

### CVSS

Common Vulnerabilities Scoring System, 用于评估漏洞严重性. CVSS v3.1 标准规定, 0.1-3.9 为低危漏洞, 4.0-6.9 为中危漏洞, 7.0-8.9 为高危漏洞, 9.0-10.0 为严重漏洞.

### CNVD/CNNVD

类似于美国的 CVE/NVD, CNVD (中国国家信息安全漏洞共享平台) 用于发布漏洞编号, CNNVD (中国国家信息安全漏洞库) 用于提供漏洞的详细分析信息.

### CBC

一种对称密码的链接工作模式, 用于加密长明文, 有迭代反馈机制, 见 [链接模式](Security/密码学/分组密码/链接模式.md)

### CTR

[CTR](Security/密码学/分组密码/链接模式.md), Counter Mode, 对称加密的一种链接模式, 支持随机访问. 

### CPA

Chosen Plaintext Attack, 选择明文攻击.

### CCA

Chosen Ciphertext Attack, 选择密文攻击. 分为 CCA1 和 CCA2 两种.

### CCA1

Lunchtime Attack, 在解密查询开始前, 就要选择所有要测试的密文, 之后不能再根据反馈重新选择.

### CCA2

Adaptive Chosen Ciphertext Attack, 攻击者基于先前解密查询的结果来选择后续的密文, 也就是根据已知信息调整策略. 比 CCA1 更强.

### CA

Certificate Authority, 用于颁发和管理密钥/数字证书的第三方机构.

### CSP

Content Security Policy, 内容安全策略, 用于防止跨站脚本 (XSS) 攻击.  

### CSRF

[Cross-Site Request Forgery, 跨站请求伪造攻击](Security/安全攻击.md), 利用用户的身份对受信网站发送恶意请求. 

### CDN

Content Delivery Network, 内容分发网络, 通过分布式缓存加速内容传输. 

---

# D

### DOM

文档对象模型, 指网页文档的编程接口, 允许脚本语言 (JS) 修改网页内容结构.

### DNS

域名系统, 用于将域名 (example.com) 变为关联 IP 地址.

### Docker

轻量容器化平台

### Device Driver

硬件驱动.

### DDL

dynamic linking library, 动态链接库.

### DSO

dynamic shared object, 动态共享对象.

### DES

早期对称加密标准, 现由于安全性已被 AES 取代. 见 [DES](Security/密码学/分组密码/Feistel-结构/DES.md)

### Digital Certificate

数字证书, 用于验证公钥拥有者身份, 见 [密钥分发与管理](Security/密码学/安全协议/密钥分发与管理.md)

### DLP

Discrete Logarithm Problem, 离散对数问题, 见 [欧拉定理](Math/数论/欧拉定理.md)

### DH

Diffie-Hellman 密钥交换, 两实体在非安全信道交换秘密的算法, 见 [DiffieHellman 协议](Security/密码学/公钥密码/DiffieHellman%20协议.md)

### DAG

Directed Acyclic Graph, 有向无环图数据结构.

---

# E

### ELF

executable linkable format, 可执行可连接格式. 见 [Unix-ELF](Compiler/Linking/Unix-ELF.md)

### Environment Variables

环境变量, 指 SHELL 运行的关键共享变量.

### Exit Code

程序退出码.

### EAT

export address table, 导出地址表.

### EDB

Exploit Database. 公开发布一些漏洞的详细信息和利用代码, 每个漏洞利用代码以 EDB-ID 标识. CVE 相比之下, 默认隐藏了漏洞细节, 最多能提供一些链接.

### ElGamal

一种基于 DLP 困难性的公钥密码算法, 包括加密和签名. 见 [ElGamal 协议](Security/密码学/公钥密码/ElGamal%20协议.md)

### EOL

End of Life, 软件或硬件停止维护的状态. 

---

# F

### Fuzzing

高效软件模糊测试框架, 见 [Fuzzing Survey](Security/软件安全测试/相关研究/Fuzzing%20Survey.md)

### FHS

file hierarchy standard, 文件层次结构标准. 见 [Linux 系统目录](System/File%20System/Linux%20系统目录.md)

### Function Signature

函数签名. 见 [compiler/linking/符号](Compiler/Linking/符号.md)

### Finite State Machine

有限状态自动机. 见计算理论.

### Frame Pointer

帧指针.

### FAC

Factorization, 因数分解. 通常指 RSA 基于的大整数分解困难问题.

---

# G

### GUI

计算机的用户图形界面

### Git

分布式版本控制系统, 用于跟踪文件更改和人员间协作. 和常见远程托管平台进行交互, 如: Github, GitLab

### Generics

泛型, 即允许接口同时复用于多种数据类型, 使用类型参数化实现, 在编译时确定具体数据类型 (即编译器为不同数据类型的调用生成不同代码, 而不是真的动态类型). 如 CPP 模板.

### GOT

global offset table, 全局偏移表

### Grammar Parser

语法分析器. 见 [编译过程](Compiler/编译过程.md)

---

# H

### HTTP

Hypertext Transfer Protocol, 应用层无状态网络协议, 基于传输层 TCP 协议, 端口为 80.

### HTTPs

HTTP+SSL/TLS, 对 HTTP 协议提供安全加密.

### Handle

句柄

### Heap

堆, 数据结构. 见 [binary heap](Algorithm/树/binary%20heap.md). 也指进程内存空间的一种结构.

### Hook

钩子.

### HMAC

基于密码学杂凑函数的消息认证码 (MAC), 是消息认证码最常见形式, 见 [HMAC](Security/密码学/消息摘要/消息认证码/HMAC.md)

---

# I

### Image File

映像文件.

### IDE

集成开发环境

### I/O Bound

I/O 密集型.

### Interrupt

操作系统中断, 见 [中断](System/Process/中断.md)

### ISR

interrupt service routine, 中断处理程序.

### Internet Standards

互联网标准, 为全球互联网互操作性和统一性提供规范和指南. 内容包括各层网络协议, 密码学和安全, 数据交换格式 (JSON) 及标记语言 (HTML).

### IV

Initialization value. 设置加密算法或加密模式的初始状态的输入参数. IV 也有引入同步和方差 (cryptographic variance). *注意, 全称 Initialization Vector 在 RFC 4949 中被标记为废弃.*

### IaaS

Infrastructure as a Service, 一种云服务交付模型.

### IDS 

Intrusion Detection System, [入侵检测系统](Network/防火墙/IDPS.md), 用于检测网络或系统中的恶意活动.  

### IPS

Intrusion Prevention System, [入侵防御系统](Network/防火墙/IDPS.md), 与 IDS 类似, 但同时具有阻止威胁的能力. 

### IoT

Internet of Things, 物联网, 通过互联网将设备连接起来实现智能化控制. 

---

# J

### JSON

JavaScript Object Notation, 轻量级数据交换格式, 易于解析和阅读.

---

# K

### Kernel Mode

操作系统内核模式.

### Kali

基于 Debian 的 Linux 发行版, 提供了内置网络安全工具.

### KMS 

Key Management Service, 密钥管理服务, 用于管理加密密钥的创建和存储. 

### KDF

密钥派生函数

---

# L

### LLVM

low level virtual machine, 编译器框架.

### Lazy Binding

延迟绑定.

### Linking

链接, 程序编译的步骤. 见 [静态链接](Compiler/Linking/静态链接.md)

### LSB

linux standard base.

### Little-endian

小端字节序, 见 [端序](HardWare/端序.md)

---

# M

### MIPS

Million Instructions Per Second, 用于衡量 CPU 每秒执行指令的速度.

### MMU

memory management unit, 内存管理单元.

### MSB

most significant bit/byte, 最大影响位.

### Multiprogramming

多道程序技术.

### Mutex

多线程互斥量. 见 [进程与线程](System/Process/进程与线程.md)

### MAC

Message Authentication Code, 消息认证码, 用于验证消息的完整性和来源真实性, 见 [MAC](Security/密码学/消息摘要/消息认证码/MAC.md)

### MD5

Message Digest Algorithm 5, 广泛使用的散列函数, 由于安全问题已被淘汰, 见 [MD-5](Security/密码学/消息摘要/MD%20迭代结构/MD-5.md)

### MITM

Man-in-the-Middle Attack, 中间人攻击, 攻击者在通信双方之间进行拦截和篡改. 

### MFA

Multi-Factor Authentication, 多因素认证, 结合多个身份验证方法以提高安全性. 

---

# N

### Name Mangling

符号改编.

### Namespace

命名空间.

### Northbright

北桥, 计算机硬件架构组成部分. 见 [计算机架构](HardWare/计算机架构.md)

### NVD

National Vulnerability Database, 由 NIST 维护的国家漏洞数据库. 为每个 CVE 漏洞提供详细评分, 影响和补丁信息. 用于对 CVE 中漏洞的补充和扩展.

### Nonce

Number Once, 用于密码算法和安全协议的一次性数字, 用于防范重放攻击.

### NAT 

[Network Address Translation, 网络地址转换](Network/防火墙/NAT.md), 用于在局域网和公网之间转换 IP 地址. 

### NIC

Network Interface Card, 网络接口卡, 用于设备与网络的连接.  

### NTP

Network Time Protocol, 网络时间协议, 用于同步计算机系统的时钟. 

---

# O

### Object File

目标文件, 编译过程中中间文件. 见 [Unix-ELF](Compiler/Linking/Unix-ELF.md)

### ORM

Object-Relational Mapping, 软件开发概念, 将面向对象的类资源/方法和一个关系型数据库绑定起来, 对类操作即等价于对数据表操作. 类的元类等价于数据库表, 一个类实例等价于表中的一行.

### OO

object-orientation, 面向对象编程.

### OTP

One-Time Password, 用于单次登录的临时密码. TOTP (Time-based OTP) 基于当前时间戳生成密码, 常用于服务器的双因素认证 (2FA) 中, 使服务器和客户端共享可不断更新的密码. HOTP (HMAC-based OTP) 指基于加密的哈希认证码的单次密钥.

### OAuth

[OAuth](Network/应用层/Authentication/OAuth.md), 一种开放标准授权协议, 允许用户授权第三方访问其资源而无需暴露凭据. 

---

# P

### Package

程序打包.

### PAE

physical address extension. 物理地址扩展.

### Page Fault

内存缺页段错误.

### Paging

分页技术, 见 [分页技术](System/Memory/分页技术.md)

### P-Code

P 码, 一种编译器中间码.

### PE

portable executable, Windows 可执行文件格式. 源于 COFF.

### PEM

Privacy-Enhanced Mail, 一种文件格式, 通常用于存储加密密钥和证书. 

### Physical Page

物理内存页, 也叫页框 Frame.

### PIC

position-independent code, 地址无关代码. 见 [动态链接](Compiler/Linking/动态链接.md)

### PIE

position-independent executable, 地址无关可执行文件.

### PLT

procedure linkage table. 过程链接表. 见 [动态链接](Compiler/Linking/动态链接.md)

### Preemption

抢占式调度. 见 [进程调度](System/Process/进程调度.md)

### Poll

轮询结构. 并发程序模型中, 某个线程阻塞等待其他资源, 线程将不时主动查询资源是否可用来结束轮询状态.

### Process

计算机进程. 见 [进程与线程](System/Process/进程与线程.md)

### Priority Schedule

进程优先级调度. 见 [进程调度](System/Process/进程调度.md)

### POSIX

Portable Operating System Interface. [IEEE P1003.1] 定义了一系列操作系统的标准接口, 被广泛应用于各类 Unix 系统.

### PGP

[PGP](Network/应用层/PGP.md), Pretty Good Privacy, 用于加密和签名的公钥加密程序. 

---

# Q

### OSI

一种安全框架, 定义了安全攻击/安全机制/安全服务, 见 [安全模型/OSI](Security/ReadMe.md)

---

# R

### Round Robin

时间片轮转法调度. 见 [进程调度](System/Process/进程调度.md)

### Read-Write Lock

读写锁. 见 [进程同步与互斥](System/Process/进程同步与互斥.md)

### Reentrant

可重入.

### Relocation

地址重定位.

### Runtime

程序运行时.

### PaaS

Platform as a Service. 同见 IaaS, SaaS.

### PRNG

Pseudo-Random Number Generator, 伪随机数生成器, 见 [流密码](Security/密码学/流密码与伪随机数/ReadMe.md)

### PKI

Public Key Infrastructure, 公钥基础设置, 见 [密钥分发与管理](Security/密码学/安全协议/密钥分发与管理.md)

### PKCS

Public-Key Cryptography Standards, 公钥密码学标准. 由原 RSA Data Security Inc. 发布的一系列**可不断修改和更新的**标准:

- [PKCS #1](Security/密码学/公钥密码/RSA/PKCS1.md): RSA 加密和签名, 包括 RSA OAEP 和 PSS 填充方案.
  - v1.5 (1993): v1.5 填充算法, 不安全.
  - v2.0 (1998): [OAEP (Optimal Asymmetric Encryption Padding)](Security/密码学/公钥密码/RSA/PKCS1.md) 加密填充算法, 推荐.
  - v2.1 (2002): PSS (Probabilistic Signature Scheme) 签名填充算法, 推荐.
- PKCS #7: 定义加密和签名消息的语法 (Syntax).

### ROS

Robot Operating System, 开源机器人操作系统.

### Reflection

反射, 在**运行时**操作和配置对象及其属性, 在编写代码时这些类和方法可能不是已知的, 常用于实现框架和库. 如 Python 的 [元编程](Language/Python/元编程.md).

### RTT

网络报文往返时间.

### RSA

目前广泛使用的公钥加密算法, 见 [RSA](Security/密码学/公钥密码/RSA/RSA.md).

### RCE

Remote Code Execution, 远程代码执行漏洞, 允许攻击者在目标系统上运行任意代码. 

---

# S

### SAML

[SAML](Network/应用层/Authentication/SAML.md), Security Assertion Markup Language, 用于单点登录 (SSO) 和身份联合的标准. 

### SQL

数据库结构化查询语言, 见 [Data Storage/SQL](Information/数据库/SQL/ReadMe.md).

### SQL Injection 

[SQL 注入攻击](Security/安全攻击.md), 利用 SQL 查询中的输入漏洞执行恶意代码. 

### Soundness

健全性, 一个逻辑系统是健全的, 当它所有推导都仅产生真实结论; 即不会错误地证明假命题为真. 见 [可靠性与完备性](Math/计算理论/可靠性与完备性.md).

### Semantic Analyzer

语义分析器. 见 [编译过程](Compiler/编译过程.md).

### Semaphore

信号量. 见 [进程同步与互斥](System/Process/进程同步与互斥.md).

### Shared Library

共享库.

### SDK

software development kit, 软件开发工具集.

### Southbridge

南桥. 见 [计算机架构](HardWare/计算机架构.md).

### Stack

先入后出的栈数据结构, 见 [binary heap](Algorithm/内核/list.md). 也指进程内存空间的一种结构, 见 [Linux 内存空间分布](System/Memory/Linux%20内存空间分布.md).

### Static Shared Library

静态共享库.

### Symbol Link

软链接.

### Symbol Resolution

符号决议.

### Synchronization

进程间同步, 见 [进程同步与互斥](System/Process/进程同步与互斥.md).

### Syntax Tree

语法树, 编译前端的中间产物, 见 [编译过程](Compiler/编译过程.md).

### System Call

系统调用. 见 [中断](System/Process/中断.md).

### SaaS

Software as a Service, 一种云服务交付模型.

### SHA

Secure Hash Algorithm, 密码学哈希函数 (消息摘要函数, 杂凑函数, 散列函数).

### SM4

中国标准对称密码算法, 见 [SM4](Security/密码学/分组密码/Feistel-结构/SM4.md).

### SM2

中国标准公钥密码算法, 基于 ECC, 包括: 加密算法, 数字签名, 密钥协商算法. 见 [SM2](Security/密码学/公钥密码/ECC/SM2.md).

---

# T

### TagDispatch

一种函数反射方法, 见 [运行时调用选择](Language/C++/语法工具/运行时调用选择.md#函数体).

### Time-Sharing System

分时系统.

### Time Slice

时间片.

### Thread

线程, 一种更轻量进程, 见 [System/进程与线程](System/Process/进程与线程.md).

### Token

词元.

### TOTP

Time-based One-Time Password, 基于时间的动态密码, 通常用于双因素认证. 

见 [O-OTP](#OTP).

---

# U

### URL

互联网统一资源定位符.

### Ubuntu

基于 Debian 的开源 Linux 流行发行版.

### User Mode

操作系统用户空间.

### UEFI

Unified Extensible Firmware Interface, 统一可扩展固件接口, 替代传统 BIOS 的启动系统. 

---

# V

### Virtual Address Space

虚拟地址空间. 一种管理内存的方式.

### VMA

virtual Memory Area, 代指进程拥有的某片连续虚拟内存区域. 见 [Linux 进程与内存管理数据结构](System/Process/Linux%20进程内存管理结构.canvas) 的 `vm_area_struct` 结构.

### VLAN 

Virtual LAN, 虚拟局域网, 通过逻辑隔离实现不同网络设备间的隔离. 

---

# W

### WSL

Windows Subsystem for Linux, 见 [OS/Linux Distribution/WSL](System/Distributions/WSL/配置%20-%20从此开始.md).

### Whitening

白化, 在第一轮和最后一轮加密中对密钥和明文进行**异或**操作, 旨在加强分组加密安全性. 最早用于 DES-X 加密算法.

### WAF

Web Application Firewall, Web 应用防火墙, 用于保护 Web 应用免受攻击. 

---

# X

### XSS 

Cross-Site Scripting, [跨站脚本攻击](Security/安全攻击.md), 通过在网页中插入恶意代码进行攻击. 

---

# Y

### YAML

类似 JSON 的数据交换格式, 但语法更复杂, 由多层嵌套键值对等组成.

---

# Z

### Zero-Day 

0Day, 零日漏洞, 指在被发现后立即被利用且未有修复措施的漏洞. 
