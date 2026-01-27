RAII, Resource Acquisition is Initialization, 资源获取即初始化. 

Resource 是指用来管理程序功能的对象, 如内存块, 文件句柄, 未释放的锁, 网络套接字, 数据库句柄等. RAII 用于避免裸露的资源句柄操作. **RAII 意味着, 对象的生命周期应在该变量的作用范围之内, 一旦 C++ 变量超出作用范围, 析构器 (destructor) 就会自动释放其资源.** 从这一点来说, RAII 更应该叫 "Scope-Bound Resource Management".

对于本地变量:
```cpp
{ // local resource 
	fp = ...;
} // call fp's destructor automatically
```

对于类成员: 使用 RAII 后, 资源显式声明, 在超出作用域时隐式释放. 比如, 循环的每次迭代都是一个新的作用域 (scope).
```cpp
// without RAII
RawResourceHandle* handle=createNewResource();
handle->performInvalidOperation();  // exception thrown; or error exit 
...
// due to exception, never gets called so the resource leaks.
deleteResource(handle); 
```

使用 RAII 可以保证在程序不同结束路径下, 对象析构函数都会被调用:
```cpp
// with RAII
class ManagedResourceHandle {
public:
   ManagedResourceHandle(RawResourceHandle* rawHandle_) 
   : rawHandle(rawHandle_) {};
   ~ManagedResourceHandle() {delete rawHandle; }
   ... // omitted operator*, etc
private:
   RawResourceHandle* rawHandle;
};

ManagedResourceHandle handle(createNewResource()); // 获取即初始化
handle->performInvalidOperation(); // 作用域结束即释放
```

[smart-pointer](smart-pointer.md)也是 RAII 原则的良好实践:
- `unique_ptr`.
- `shared_ptr`, 注意 `auto_ptr` 应被废弃.

使用 RAII 原则后, 对象的释放/获取直接和生命周期相关, 不必在逻辑中额外添加对应的释放操作, 对象生命周期结束后即**自动**调用析构函数. 此类对象的使用, 和直接使用 C 栈变量同样简洁, 但需注意析构函数不会自动释放堆成员 (如 new 或 malloc), 此时最好使用*智能指针*. 

PS: 不要依赖 C++ 编译器的任何默认/隐式行为. C++ 的细节复杂度极逆天, 脑子是记不住的 (也不要试图把脑子训练成人肉编译器), 应确保所有行为都是准确和明确定义的. 如果不确定编译器是否会自动初始化或自动释放资源, 最*简单*的方式就是手动定义初始化和释放方法.

在带 GC 语言中, 常见错误是不习惯手动释放底层对象, 导致资源泄露. python 用[上下文管理器](../../Python/开发工具/contextlib.md)解决这个问题. 

## 例子

考虑文件句柄的使用. 首先假定一切顺利进行:

```cpp
void doSomethingWithAFile(const char* filename) {

    FILE* fh = fopen(filename, "r"); // 以只读模式打开文件

    doSomethingWithTheFile(fh);
    doSomethingElseWithIt(fh);

    fclose(fh); // 关闭文件句柄
}
```

但是各个函数都可能执行失败, 并返回错误代码. 所以引入了很多和业务逻辑无关的代码:

```cpp
bool doSomethingWithAFile(const char* filename){
    FILE* fh = fopen(filename, "r"); // 以只读模式打开文件
    if (fh == nullptr) // 执行失败, 返回的指针是 nullptr
        return false; 

    // 假设每个函数会在执行失败时返回false
    if (!doSomethingWithTheFile(fh)) {
        fclose(fh); // 关闭文件句柄, 避免造成内存泄漏
        return false;
    }
    if (!doSomethingElseWithIt(fh)) {
        fclose(fh); // 关闭文件句柄
        return false; // 反馈错误
    }

    fclose(fh); // 关闭文件句柄
    return true; // 指示函数已成功执行
}
```

返回错误代码是 C 常用的错误处理方式. 除此之外, C 程序员还会用 `goto` 来简化错误处理:

```cpp
bool doSomethingWithAFile(const char* filename) {
    FILE* fh = fopen(filename, "r");
    if (fh == nullptr)
        return false;

    if (!doSomethingWithTheFile(fh))
        goto failure;

    if (!doSomethingElseWithIt(fh))
        goto failure;

    fclose(fh); // 关闭文件
    return true; // 执行成功

failure:
    fclose(fh);
    return false; // 反馈错误
}
```

C++ 推荐的错误处理方式是*异常*, 尽管很多底层程序员不喜欢用它, 因为复杂程序无法有效控制报错在哪一层触发, 又在哪一层被处理.

```cpp
void doSomethingWithAFile(const char* filename){
    FILE* fh = fopen(filename, "r"); // 以只读模式打开文件
    if (fh == nullptr)
        throw std::exception("Could not open the file.");

    try {
        doSomethingWithTheFile(fh);
        doSomethingElseWithIt(fh);
    }
    catch (...) {
        fclose(fh); // 保证出错的时候文件被正确关闭
        throw; // 然后, 重新抛出这个异常, 交给上层.
    }

    fclose(fh); // 关闭文件
    // 所有工作顺利完成
}
```

但 C++ 的 fstream 有 RAII 机制, 在退出作用域时, 自动调用析构函数来关闭句柄.
```cpp
void doSomethingWithAFile(const std::string& filename){
    // ifstream == input file stream
    std::ifstream fh(filename); // 打开一个文件

    // 对文件进行一些操作
    doSomethingWithTheFile(fh);
    doSomethingElseWithIt(fh);

} // 文件已经被析构器自动关闭
```

## 参考

[What is meant by Resource Acquisition is Initialization (RAII)? Stack Overflow](https://stackoverflow.com/questions/2321511/what-is-meant-by-resource-acquisition-is-initialization-raii)

[Resource acquisition is initialization Wikipedia .](https://en.wikipedia.org/wiki/Resource_acquisition_is_initialization)