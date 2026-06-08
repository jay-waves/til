## 模板模式

## 策略模式

## 观察者模式

观察者模式 (Observer Pattern) 定义了*一对多的依赖关系*, 当 "一" 的状态发生改变时, 所有依赖于 "一" 的对象都会得到通知并更新.

```cpp
class Observer {
public:
	virtual ~Observer() {}
	virtual void update(....) = 0;
};

class Subject {
public:
	virtual ~Subject() {}
	virtual void registerObserver(Observer* ob) = 0;
	virtual void notifyObservers() = 0;
};

/*
 for observers of subject:
	 for observer in observers:
		 observer.update();
*/
```

## 备忘录模式

备忘录模式 (Memento Pattern) 在不破坏封装的前提下, 捕获并保存一个对象的内部状态, 必要时恢复到该状态.
- 发起人 (Originator): 创建并恢复自身的状态快照
- 备忘录 (Memento): 存储发起人状态, 通常只允许发起人自行访问.
- 管理者 (Caretaker): 保存备忘录, 但不关心和修改内容.

```cpp
class Memento {
private:
	T state_;
	// 只有发起人可访问
	friend class TextEditor;

public:
	Memento(const T& s) : state_(s) {}
};

class TextEditor {
private:
	std::string text_;
public:
	TextEditor() : text_("") {}

	Memento* createMemento() {
		return new Memento(text);
	}
	
	void restoreFromMemento(Memento* meme) {
		if (meme) {
			..
		}
	}
};

class Caretaker {
private:
	vector<Memento*> memes_;
public:
	Caretaker() {}
	~Caretaker() {
		// 负责管理内存
		for (auto& meme : memes_)
			delete mem;
	}
	
	void saveMemento(Memento* m) {
		memes_.push_back(m);
	}
	...
};

caretaker.saveMemento(editor.createMemento());
```


## 命令模式

## 访客模式

访问者模式 (Visitor Pattern) 将数据和操作分离, 在不改变数据结构的情况下, 定义对该结构的新的操作.
- 元素 (Element): 定义一个接受访问者的接口 `accept()`
- 访问者 (Visitor): 声明对每种元素类型的访问操作 `visit()`
- 双重分派 (Double Dispatch)

```cpp
class Visitor;
class Element {
public:
	virtual ~Element() {}
	virtual void accept(Visitor* vis) = 0;
}

class Visitor {
public:
	virtual ~Visitor() {}
	virtual void visit(XXXElement* elm) = 0;
	virtual void visit(YYYElement* elm) = 0;
};


void XXXElement::accept(Visitor* vis) /* override */ {
	vis->visit(this);
}
```

## 责任链模式

责任链模式 (Chain of Responsibility Pattern) 将请求的发送者和接收者解耦, 层层将请求解耦, 将责任向下传递.

```cpp
class Handler {
protected:
	Handler* next_handler_;
public:
	virtual void handleRequest() = 0;
protected:
	void passToNext() {
		if (next_handler_)
			next_handler_->handleRequest();
	}
};

class HandlerA : public Handler {
public:
	void handleRequest() override {
		passToNext();
	}
};
```

## 迭代器模式

```cpp
template<typename T>
class Container {
private:
	std::vector<T> items_;

public:
	Container(std::vector<T> data) : items_(std::move(data)) {}

	class Iterator {
	public:
		using iterator_category = std::forward_iterator_tag;
		using value_type = T;
		using difference_type = std::ptrdiff_t;
		using pointer = T*;
		using reference = T&;

		Iterator(typename std::vector<T>::iterator it) : current(it) {}

		Iterator& operator*() const { return *current; }
		Iterator& operator++() { ++current; return *this; }
		Iterator operator++(int) {Iterator tmp = *this; ++current; return tmp; }
		bool operator==(const Iterator& other) const { return current == other.current; }
		bool operator!=(const Iterator& other) const { return current != other.current; }
	private:
		typename std::vector<T>::iterator current;
	};

	Iterator begin() { return Iterator(items.begin()); }
	Iterator end() { return Iterator(items.end()); }
};
```

## 解释器模式
