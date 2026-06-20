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