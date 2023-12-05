React 是一个用于构建用户界面的 JavaScript 库。以下是 React 的一些基础语法和特性：

### 1\. JSX

JSX (JavaScript XML) 是一个看起来像 XML 的 JavaScript 语法扩展。它使你能够写出类似HTML的代码片段，但实际上你正在写JavaScript。例如：

```
jsxCopy codeconst element = <h1>Hello, world!</h1>;

```

### 2\. 组件

React 应用通常由多个组件组成。组件可以是函数或类：

-   **函数组件**:
    
    ```
    jsxCopy codefunction Welcome(props) {
      return <h1>Hello, {props.name}</h1>;
    }
    
    ```
    
-   **类组件**:
    
    ```
    jsxCopy codeclass Welcome extends React.Component {
      render() {
        return <h1>Hello, {this.props.name}</h1>;
      }
    }
    
    ```
    

### 3\. Props

Props (属性) 是从父组件传递给子组件的参数。Props 是只读的。

```
jsxCopy codefunction Welcome(props) {
  return <h1>Hello, {props.name}</h1>;
}

```

### 4\. State

State 是组件的私有数据，可以随时间改变。State 通常用于存储组件的局部数据，而 Props 用于传递数据。只有类组件可以有 state，但自 React 16.8 版本后，使用 Hooks 的函数组件也可以有 state。

```
jsxCopy codeclass Clock extends React.Component {
  constructor(props) {
    super(props);
    this.state = {date: new Date()};
  }
  render() {
    return <div>{this.state.date.toLocaleTimeString()}</div>;
  }
}

```

### 5\. 生命周期方法

类组件具有多个生命周期方法，例如 `componentDidMount` 和 `componentDidUpdate`，允许你在特定时间执行代码。

### 6\. 事件处理

React 有自己的事件系统，你可以在 JSX 中直接添加事件处理程序。

```
jsxCopy code<button onClick={this.handleClick}>
  Click me
</button>
```