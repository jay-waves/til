Rust 通过**所有权机制**完成编译时对内存管理的检查. 内存管理的核心任务是管理堆上数据, 减少重复堆数据, 并及时清理避免内存泄露.
- 所有权 (Ownership): 一个变量拥有一个值, 并且一个值同一时刻只有一个有效所有者. 值跟随所有者的生命周期.
- 借用和引用 (Borrowing): 允许不可变和可变引用. 但同一时刻只有一个可变引用或多个不可变引用.
- 移动语义 (Move Semantics): 赋值或哈数传参时, 默认资源转移, 而非拷贝.

Rust 中变量离开作用域后会失效, 类似 C++ RAII, 称为 `Drop`.
```rust 
{ // s is invalid
	let s = "hello"; // s is valid
	
} // s is dropped and invalid now
```

Rust 处理**堆上数据**时, 并不进行"引用拷贝", 而是丢弃. 该行为称为 `Move`.  ==从设计上, rust 希望不对数据进行值拷贝, 也因此所有自动拷贝行为的性能开销都是可控的.==
```rust
let s1 = String::from("hello");
let s2 = s1; // from now s1 is invalid

let s3 = s2.clone(); // both s2, s3 are valid

// return or call of function also transfer ownership
takes_owner_ship(s3); // from now s3 is invalid
```

Rust 处理**纯栈上数据**时 (不能嵌套任何堆数据), 会直接进行值拷贝. 称为 `Copy`
```rust
let x = 5;
let y = x; // both x,y are valid
```

## 借用和引用

创建一个引用的过程叫 `borrowing`, 引用并没有所有权. 默认, 引用不可改变原对象.
```rust
let s1 = String::from("hello");
let len = cal_length(&s1);
```

**存在一个可变引用时, 关于该对象不能存在其他引用**, 在编译期即避免了数据竞争.

```rust
let mut s = String::from("hello");
// 错误代码:
let r1 = &s; 
let r2 = &mut s; 
println!("{} and {}", r1, r2); // error!

// 正确代码:
let r1 = &s;
println!("{}", r1); // the end of r1's lifetime
let r2 = &mut s;
println!("{}", r2); // no problem
```

Rust 编译器确保引用的值总是有效的, 即不会出现空指针错误. 当对数据的引用没有结束生命周期, 数据本身也应没有结束生命周期.
```rust
fn dangle() -> &String { // dangle returns a reference to a String

    let s = String::from("hello"); // s is a new String

    &s // we return a reference to the String, s
} // Here, s goes out of scope, and is dropped. Its memory goes away.
  // Danger!
```

## 用 C++ 进行拙劣模仿

用 `std::unique_ptr` 模仿所有权:

```cpp
#include <memory>

class Resource {
public: 
	Resource() {...};
	~Resource() {...};

void takeOwnership(std::unique_ptr<Resource> res) {
	...
}
};

std::unique_ptr<Resource> res1 = std::make_unique<Resource>();
takeOwnership(std::move(res1));
```

用 `const&` 模仿不可变借用:

```cpp
...
```

实现强制移动语义:

```cpp
class Resource {
public:
	Resource() = default;

	Resource(const Resource&) = delete;
	Resource& operator=(const Resource&) = delete;

	Resource(Resource&&) noexcept = default;
	Resource& operator=(Resource&&) noexcept = default;
};
```

Rust 的 `Option` 枚举类型要求**一个值要么存在, 要不存在 (None)**. 在其它语言中, 缺失值 (None) 需要显示处理, 缺失值可以被赋值给其他指针或对象, 变异器无法检查这种潜在错误.

使用 cpp `std::optional` 模仿 Option:

```cpp
#include <optional>

std::optional<std::string> findUser(int id) {
	if (id == 1) 
		return "Alice";
	else
		return std::nullopt; // 类似 Rust 中的 None
}

auto user = findUser(1);
if (user)  // optional 可以安全地用于判断, 显示区分空值 "" 和无值 nullptr
	...
else
	std::cout << "not found";
```


