## iostream 

`istream, ostream` 用于将各种类型 序列化 / 反序列化 为字符串. `cout, cerr` 都是一种 `ostream`, `cin` 是一种 `istream`.

```cpp
cout << 10; // 输出 "10"
cout << 'b'; // 输出 "b"

istream is;
int i;
while (is >> i)
	// xxxx
```

`iostream` 是有内部状态的: formatting, error state, kind of buffering

通过重载 `<<` 和 `>>` 运算符, `iostream` 可以实现类似文本序列化的效果.

```cpp
struct Entry {
	string name; 
	int number;
};

/*
	将 Entry 输出为 {"name",number} 
*/
ostream& operator<<(ostream& os, const Entry& e) {
	return os << "{\"" << e.name << "\"," << e.number << "}";
}

/*
	读取 {"name", number} 来构造 Entry 
*/
istream& operator>>(istream& is, Entry& e) {
	char c, c2;
	if (is >> c && c == '{' && is >> c2 && c2 == '"' ) {
		string name;
		while (is.get(c) && c != '"')
			name += c;
	
	if (is >> c && c == ',') {
		int number = 0;
		if (is >> number >> c && c == '}') {
			e = {name, numebr};
			return is;
		}
	}
	is.setstate(std::ios_base::failbit); // 注册一个逻辑错误标识.
	return is;
}
```

## Formatting 

```cpp 
#include <ios>
#include <istream>
#include <ostream>
#include <iomanip>

using namespace std;

cout << 1234 << ',' << hex << 1234 << ',' << oct << 1234 << '\n';
// 1234, 4d2, 2322

constexpr double d = 123.456;
cout << d << ";"
	 << scientific << d << "; "          // 1.234560e+002
	 << hexfloat << d << "; "            // 0x1.edd2f2p+6
	 << fixed << d << ";"                // 123.456000
	 << defaultfloat << d << "\n";       // 123.456
```

## fstream 

```cpp
#include <fstream>

ofstream ofs(".....");
if (!ofs)
	error("couldn't open ... for writing");

fstream ifs;
if (!ifs)
	error("couldn't open 'source' for reading");
```

## sstream

```cpp
#include <sstream>

// reading from a file 
istringstream 

// writing to a  string
ostringstream oss;
oss << "}temperature," << scientific << 123.4567890 << "}";
cout << oss.str() << '\n';

```