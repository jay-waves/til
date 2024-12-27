## String

C 风格字符串易错且难用, C++ 定义了 string 类. 

|          | C String                       | C++ String                            |
| -------- | ------------------------------ | ------------------------------------- |
| 类型     | `char *`                       | class string                          |
| 方法     | 无                             | `find, copy, delete, replace, insert` |
| 内存分配 | 手动管理, 长度为字符串长度加一 | 自动管理, 由类方法自动维护                                      |

## 初始化构造

```cpp
#include <string>
using std;

string();
string(const string& str);
string(const char* s);
string(int n, char c); // n * c
```

和 `char *` 相互转化:

```cpp
string str = "hello";
const char* cstr = str.c_str();

char *s = "it";
string str(s); // 基础类型可以隐式转化为高级类型
```

## 赋值

```cpp
string& operator=(const char* S);
string& operator=(const string &s);
string& operator=(char c);
string& assign(const char *s);
string& assign(const char *s, int start, int n); // 将 s 从 start 位置开始的 n 个字符赋给当前串
string& assign(const string &s);
```

## 取址

```cpp
char& operator[](int n);
char& at(int n);
```

对 string 取址是非常危险的, 包括 `const char* addr = str.c_str()` 和上述操作. 因为对 string 进行操作后 (如改变其内容), 其实际内存地址可能发生变化, 导致原本获取的地址失效. 每次访问时, 都应该获取最新地址.

### string_view

C++17 引入 `std::string_view`, 可以持续访问字符串 `string`, 而不实际拥有字符串. 即使原 `string` 重新分配, 也不会出错. 由于其不拷贝字符串, 引用地址也不会失效, 因此在切片和传递参数时很高效.

```cpp
string_view();
string_view(const char* str, size_t len);
string_view(const std::string& str);

string_view(const stirng_view& other); // 拷贝构造函数, 通过拷贝初始化新对象
string_view(string_view&& other);      // 移动构造函数, 通过窃取初始化新对象
string_view& operator=(const string_view& other);
```

## 字符串拼接

```cpp
string& operator+=(const string& str);
string& operator+=(const char* str);
string& operator+=(const char c);
```

## 字符串查找和替换

## 字符串比较

类似 `strcmp`, 在字符串字典顺序 `a > b` 时, 返回 1; `a == b` 时, 返回 0; `a < b` 时, 返回 -1.

```cpp
int compare(const string &other) const;
int compare(const char *s) const;
```

## 字符串子串

```cpp
string substr(int pos = 0, int n = pos) const; // 返回 pos 开始的 n 个字符串组成的 新 字符串.
```

## 字符串插入删除

```cpp
string& insert(int pos, const char *s);
string& insert(int pos, const string& str);
string& insert(int pos, int n, char c);
string& erase(int pos, int n = npos);
```