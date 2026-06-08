
## 工厂模式 & 抽象工厂模式

```cpp
class Shape {
};

class Circle : public Shape {
};

class Rectangle : public Shape {
};

// 通过模板实现
template <typename T>
Shape* createShape() {
	return new T();
}
```

```cpp
struct Point {
	float x, y;
	friend class PointFactory;
	
private:
	Point(float x, float y) : x(x), y(y) {}
};

struct PointFactory {

	static Point new_cartesian(float x, float y) {
		return Point{ x, y };
	}
	
	static Point new_polar(float r, float theta) {
		return Point { r*cos(theta), r*sin(theta) };
	}
};
```

## 构建者模式

显式定义创建对象的接口.
* 所有权和生命周期需要清晰.
* 定义流式接口.
* 复杂对象才考虑构建者模式, 否则用类的初始化函数即可.

禁用所有构造函数, 只暴露 Builder:

```cpp
struct Element {

	static unique_ptr<Builder> build(...) {
		return make_unique<Builder>(...);
	}
	
protected:
	Element() {}
	Element(...) : ... {}
};
```

compiste-builder: 复杂对象, 可以拆分为多个 Builder 分阶段构建.

```cpp
struct DbConfig { std::string host; int port; };
struct CacheConfig { bool enabled; };
struct AppConfig { DbConfig db; CacheConfig cache; };

struct DbBuilder {
  DbConfig cfg{};
  DbBuilder& host(std::string h){ cfg.host = std::move(h); return *this; }
  DbBuilder& port(int p){ cfg.port = p; return *this; }
  DbConfig build() && { return std::move(cfg); } // 转移所有权.
};

struct CacheBuilder {
  CacheConfig cfg{};
  CacheBuilder& enabled(bool e){ cfg.enabled = e; return *this; }
  CacheConfig build() && { return std::move(cfg); }
};

struct AppBuilder {
  AppConfig cfg{};

  AppBuilder& db(DbConfig d){ cfg.db = std::move(d); return *this; }
  AppBuilder& cache(CacheConfig c){ cfg.cache = std::move(c); return *this; }

  AppConfig build() && {
	// 可以放跨组件的校验
    // e.g. if (!cfg.cache.enabled && ...) ...
    return std::move(cfg);
  }
};

// 用例
auto app = AppBuilder{}
  .db(DbBuilder{}.host("x").port(5432).build())
  .cache(CacheBuilder{}.enabled(true).build())
  .build();
```

Builder 和 Factory 的主要区别是, Factory 一次生产完成, Builder 需要逐步提供信息.

## 单例模式

一个类只有一个对象, 一次创建多次使用. 构造步骤:
1. 类构造函数私有化
2. 增加静态私有的当前类的指针变量
3. 提供静态对外接口, 让用户获得单例对象

在程序创建时立即创建该单例对象, 线程安全性更高, 内存消耗更大. 称为*饿汉式*.

```cpp
// 饿汉
class Singleton {
private:
	Singleton() {
		...
	}
	static Singleton instance;

public:
	Singleton(cosnt Singleton&) = delete;
	Singleton& operator=(const Singleton&) = delete;

	static Singleton& getInstance() {
		return instance;
	}
};

// 在启动时初始化静态成员变量
Singleton Singleton::instance; 
```

在第一次使用时才创建实例, 避免不必要消耗, 但线程安全性低. 称为*懒汉式*.

```cpp
class Singleton {
private:
	Singleton() {
		
	}
	static Singleton* instance;
	static std::mutex mutex;

public:
	Singleton(const Singleton&) = delete;
	Singleton& operator=(const Singleton&) = delete;

	static Singleton& getInstance() {
		if (instance == nullptr) {
			std::lock_guard<std::mutex> lock(mutex); // 加锁
			if (instance == nullptr) { // 双重检查
				instance = new Singleton();
			}
		}
		return *instance;
	}
	
	~Singleton() {
		delete instance;
		instance = nullptr;
		...
	}
}

Singleton* Singleton::instance = nullptr;
std::mutex Singleton::mutex;
```

对于 C++11 以后版本, **静态局部变量也有线程安全初始化机制**. 最简洁. 但后续的读写操作不保证线程安全.

```cpp
class Singleton {
private:
	Singleton() {
		...
	}
public:
	Singleton(const Singleton&) = delete;
	Singleton& operator=(const Singleton&) = delete;

	static Singleton& getInstance() {
		static Singleton instance;
		return instance;
	}
	...
}
```

## 原型模式

原型模式, 是指直接复制某个已有实例, 用于构造新实例. 提供 `clone()` 方法时, 需要注意区分浅拷贝和深拷贝的区别.

```cpp
class Monster {
public:
    virtual ~Monster() = default;

    // 原型模式构建新对象的方式: 拷贝构造
    virtual Monster* clone() const = 0;

    void setHP(int hp) { hp_ = hp; }
    void setAttack(int attack) { attack_ = attack; }

protected:
    Monster(int hp, int attack) : hp_(hp), attack_(attack) {}

    int hp_;
    int attack_;
};

class Goblin : public Monster {
public:
    Goblin(int hp, int attack) : Monster(hp, attack) {}

    Monster* clone() const override {
        return new Goblin(*this); // 自动生成浅拷贝函数
    }

};

class Dragon : public Monster {
public:
    Dragon(int hp, int attack, const std::string& breathWeapon)
        : Monster(hp, attack), breathWeapon_(new std::string(breathWeapon)) {}

    // 自定义拷贝构造函数，执行深拷贝
    Dragon(const Dragon& other) : Monster(other), breathWeapon_(new std::string(*other.breathWeapon_)) {}

    ~Dragon() override { delete breathWeapon_; }

    Monster* clone() const override {
        return new Dragon(*this); // 调用拷贝构造函数
    }

private:
    std::string* breathWeapon_;
};

```

在拥有"反射"的编程语言中, 复杂类型也很容易被序列化/反序列化. 因此, 原型模式也可以基于序列化后的二进制来做, 但是 C++26 才计划支持反射.

