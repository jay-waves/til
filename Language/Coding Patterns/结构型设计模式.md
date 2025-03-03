## 桥接模式

桥接模式的目的是将对象和对象操作类分离, 通过抽象基类桥接, 各自可以独立变化. 
- Abstraction -> Implementor. 抽象类别及抽象实现.
- Refined Abstraction -> Concrete Implementor. 具体类别及实现.

```cpp
// Implementor
class DrawingAPI {
public:
	virtual void drawCircle(int x, int y, int radius) = 0;
}

// Concrete Implementor
class SVGDrawing : public DrawingAPI {
public:
	void drawCircle(int x, int y, int radius) override {
		...
	}
}

class V12Drawing : public DrawingAPI {
public:
	void drawCircle(int x, int y, int radius) override {
		...
	}
}

// Abstraction
class Shape {
protected:
	DrawingAPI* drawingAPI;
public:
	Shape(DrawingAPI* drawingAPI) : drawingAPI(drawingAPI) {}
	virtual void draw() = 0;
}

// Refined Abstraction 
class Circle : public Shape {
private:
	int x, y, radius;
public:
	Circle(DrawingAPI* drawingAPI, int x, int y, int r) : 
		Shape(drawingAPI), x(x), y(y), radius(r) {}
	void draw() override {
		drawingAPI->drawCircle(x, y, radius);
	}
}
```

## 适配器模式

## 装饰器模式

## 组合实体模式

## 享元模式

## 外观模式

## 代理模式