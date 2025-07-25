## 错误处理

- 基于返回值
- 抛出异常
- [`tl::optional`](https://github.com/TartanLlama/optional) --> C++20 `std::optional`, 类似 Rust `Option`
- [`tl::expected`] --> C++23 `std::expected`, 类似 Rust `Result`
- outcome, 类似 Rust `anyhow, thiserror`

## 变量声明

```c
void foo() {
	MyClass obj();            // 这是函数类型 (Most Vexing Parse, 解析歧义)
	MyClass obj1 = MyClass(); // 这是一个 MyClass 对象
	MyClass obj2{};           // 这是一个 MyClass 对象
}
```

## StackTrace 

## range-v3

C++20 Ranges. 函数式迭代器.

## 序列化

- JSON: jsoncpp, nlohmann/json 
- YAML: yaml-cpp 
- cereal, bitsery. 结构体等二进制序列化
- protobuf: 跨语言二进制序列化格式

## 单元测试

- benchmark 
- Catch2 
- doctest 

## 其他开发工具库:

- cxxopts: 命令行参数解析
- spdlog: 日志