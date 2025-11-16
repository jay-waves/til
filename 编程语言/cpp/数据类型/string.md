## String

C 风格字符串易错且难用, C++ 定义了 string 类. 

|          | C String                       | C++ String                            |
| -------- | ------------------------------ | ------------------------------------- |
| 类型     | `char *`                       | class string                          |
| 方法     | 无                             | `find, copy, delete, replace, insert` |
| 内存分配 | 手动管理, 长度为字符串长度加一 | 自动管理, 由类方法自动维护                                      |

C++ String 是单字节字符串序列, 无内置编码模式. 

```cpp
template<typename Char>
class basic_string {
	//....
};

using string = basic_string<char>
```

C++ string 不保证结尾附带终止符 `\0`, 也不保证内部没有 `\0` 字符. 但如果需要和 C 风格 API 交互, 必须保证结尾有 `\0`, 否则会错误.
- 如果 string 内部包含 `\0`, 会导致 `c_str()` 返回长度终止在第一个 `\0` 处.
- C++11 后, `c_str()` 和 `data()` 的结果都会自动附加一个 `\0`, 这个字符不计入总长度.

## 字面量

单引号标注*字符字面量 (character literal)*, 双引号标注*字符串字面量 (string literal)*. 

```cpp
char c = 'A'; // 实际等价于一个整数常量

int x = 'AB';
int y = '你'; // 多字符字面量, 使用 int 来存储, 将编码拼接在同一个 int 中, 超过 4 字符就是未定义行为.
// 这种方式并不可靠, 其实际编码方式取决于源码文件的字符编码.

const char *s = "A"; // 字符串字面量, s 指向 {'A', '\0'}
```

其他字符串类型:
- `u8"AAAAA"` UTF-8 字符串, 等价于 `const char8_t[]`
- C++20 引入了 `u8string`, 内部使用 `char8_t`. 但没有提供任何编码转换功能, 可使用本地化库 ICU.
- `u"AAAAAA"` UTF-16 字符串, 等价于 `const char16_t[]`
- `U""` UTF-32 字符串, 等价于 `const char32_t[]`
- `L""` 宽字符串, `const wchar_t[]`

## 初始化构造

构造 `string` 时, 短字符串会直接存放在栈上, 长字符串才放在堆上. 这样的优点是减少内存碎片化.

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

// 错误的做法:
const char* p = std::string("hello").c_str();
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

string& append(const char *s);
string& append(const string &s); // 等价于 operator+=()
string& append(const string &s, int pos, int n);

```

## 字符串查找和替换

```cpp
int find(const string& str, int pos = 0) const; // str 第一次出现的位置, pos 指明开始查找的位置
int find(const char*s, int pos = 0) const;
int rfind(const string& str, int pos = npos) const; // str 最后一次出现的位置
string& replace(int pos, int n, const string& str);
string& replace(int pos, int n, const char *s);
```

## 字符串比较

类似 `strcmp`, 在字符串字典顺序 `a > b` 时, 返回 1; `a == b` 时, 返回 0; `a < b` 时, 返回 -1.

```cpp
int compare(const string &other) const;
int compare(const char *s) const;
```

## 字符串子串

```cpp
string substr(int pos = 0, int n = npos) const; // 返回 pos 开始的 n 个字符串组成的 新 字符串.
```

## 字符串插入删除

```cpp
string& insert(int pos, const char *s);
string& insert(int pos, const string& str);
string& insert(int pos, int n, char c);
string& erase(int pos, int n = npos);
```