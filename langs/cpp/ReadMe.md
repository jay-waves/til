## History

C++11
- `unique_ptr`, `shared_ptr`, `make_shared`
- `auto`
- lambda function
- functor, `funtion`
- `nullptr`
- rvalue reference `&&`, `move()`
- `thread`, `mutex`, `condition_variable`
- `valarray`

C++14
- generic lambda `auto lambda = [](auto x) { return x + x; };`
- `make_unique`


C++17
- `optional`, `nullopt`
- structured bindings `auto [a, b] = ...`
- `if constexpr`
- parallel algorithms
- `variant`, `visit`

C++20
- coroutines
- concepts
- ranges
- modules

C++23
- `expected`
- `ranges::to`
- `print`
- `stacktrace`

## Third-Party

操作系统相关跨平台库:
- [gulrak/filesystem](https://github.com/gulrak/filesystem)

序列化: 见 [C++/开发工具](../../tools/readme.md)

错误处理: 见 [C++/开发工具](../../tools/readme.md)

GUI:
- QT 
- wxWidgets 
- ImGUI 
- CEF / Chromium Embedded Framework 
- [clay](https://github.com/nicbarker/clay)
- [raylib](https://github.com/raysan5/raylib)

通用大的工具框架:
- Boost 
- abseil (google): 主要是对 STL 的补充
- folly (meta): 也是对 STL 的补充, 以及高性能网络库
- Qt: 也有很多框架工具
- Poco 

通信与网络框架:
- gRPC: google 系列工具, 和 protobuf 搭配
- asio 
- libuv: 比 libevent 更新.

数学:
- gmp 
- Eigen
- SIMD (Vc)

## Zero Cost Abstraction

> C++ obey the zero-verhead principle: You don't pay what you don't use. And further: you couldn't hand code any better what you do use. --Bjarne

**零开销抽象**意味着, 添加一个高级编程概念或语法时, 额外的开销会在编译期被消化, 不会带来额外的运行时开销, 即:
- no overhead in space
- no overhead in runtime

运行时的零开销抽象很难(几乎不可能)实现, 参考 [Cppcon 2019](https://www.youtube.com/watch?v=rHIkrotSwcc). C++ 只是保证, 未使用的功能不会造成实际开销, 被使用的功能的大部分额外(隐式的)开销不发生在运行时, 副作用是更长的编译时间. 

## Reference

[Learn Modern C++, discover a launguage matched to today's computing needs](https://learnmoderncpp.com/) - cpptutor

*A Tour of C++*. Bjarne Stroustrup.

https://github.com/andreasfertig/cppinsights/