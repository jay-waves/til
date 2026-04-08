## JavaScript 

JS 只有双精度浮点数一种数值类型。

```js
3;
1.5;
```

其他类型

```js
Infinity; // result of e.g. 1/0
-Infinity;
NaN; 

true;
false;

// equality 
1 === 1;
2 === 2;
1 !== 2;

// fasle, null, undefined, NaN, 0 and "" are falsy; everything else is truthy 
null;
undefined;

// Arrays are list of any type, with variable length
var arr = ["hello", 45, true];

// Objects are equivalent to maps or dictionaries 
var obj = {key1: "Hello", key2: "World"};

function my_func(thing) {
	return ....;
}

if (true){
    var i = 5;
}
i; // = 5, only function in js has blocked scope
```

原生支持反射：

```js 
var arr = [12, 34]
arr.join(";"); // --> "1234"
```

ES6:

```js

let name = "Bob" // 使用 let 定义时，有词法作用域

name = "William" // 用 let 定义的变量可以被重复赋值

const pi = 3.14; 

// lambda 
const is_even = (number) => {
	return number % 2 === 0;
};

is_even(7); // false 
```

## TypeScript 

TS 定义了三个基础类型：

```ts 
let done : boolean = false;
let num  : number  = 42;
let name : string  = "aaa"
```

其他类型：
```ts
let list : Array<number> = [1, 2, 3]
enum Color { Red, Green, Blue };
```

函数：

```ts 
let f1 = function (i : number) : number { return i * i; }
let f2 = (i : number) : number => { return i * i; }
let f5 = (i : number) => i * i;
```

接口：

```ts
interface Person {
	name : string;
	age?: number;   // 可选值
	move() : void;  // 函数签名（不考虑函数名）
}
```

类：

```ts
class PointPerson implements Person {
	name: string 
	move() {}
}

let p1 = new Point(10, 20);

// 泛型类与接口
class Tuple<T1, T2> {
  constructor(public item1: T1, public item2: T2) {
  }
}
```

## 参考

[learn Typescript in Y minutes](https://learnxinyminutes.com/typescript/)

[TypeScript is JavaScript with syntax for types](https://www.typescriptlang.org/)