> LeBlanc's Law: Later equals never


Don't Repeat Yourself

> Every piece of knowledge must have a single, unambiguous, authoritative representation within a system

函数不要超过 20 行
- 性能不是借口. 
- 避免无意识的垃圾累积. 代码改动不能让代码看起来更糟糕.

> 让营地比你来时更干净 -- 童子军军规

类应仅有单一职责, 避免巨无霸类.

使用封装避免长参数列表.
- 参数变动频率相同, 封装为同一个类.
- 变化频率不同时, 静态不变的应成为当前类的一部分.
- 其他分开封装. 
- 以抽象的对象取代基本类型, 选择合适的集合类型. (Primitive Obsession)

避免嵌套, 即避免过深缩进.
- 避免 else 语句, 通过写法的改进.
- 避免循环语句

封装掉类内部的细节. 比如类不应提供较多 `getter()` 方法, 暴露在外的接口只应是功能性的, 而不是直接暴露类内的数据. 即, 隐藏委托关系 (Hide Delegation), 将调用封装起来.

```
# 过长消息链 (message chains), 也被称为 火车残骸 (train wreck)
# 一个对象请求另一个对象, 后者再次请求另一个对象, 导致一长串的取值和委托
book.get_info().get_author().get_name()

book.get_author_name()
```

移除设值函数 (remove setting method), 即删除 `setter()` 方法. 其本质是, 可变数据 (mutable data) 是易错的. 在函数式编程中, 数据来源是不可变的, 更新时总产生新的数据副本, 这样避免了未知来源对原始数据的大意修改.
- 所有字段只在构造函数中初始化
- 所有方法都是纯函数
- 如果需要有改变, 返回一个新对象, 而不是修改已有字段.
- 移除全局数据, 因为全局变量总是可变的.
- 区分类的性质, 实体对象限制数据变化, 而值对象要设计为不变类.

一次性完成变量初始化, 即总是将变量的声明和赋值部分结合在一起. 让代码变成声明式代码, 讲 "做什么"; 而不是命令式代码, 总是说 "怎么做", 从而将意图和实现分开.

高层次对象和低层次对象间需要防腐层, 该层阻止高低层次间直接的对象传递, 从而避免耦合. 具体实现和业务代码应隔离, 业务代码抽象程度高, 因此总应该更加稳定, 避免依赖于不稳定的具体实现.

> High-level modules should not depend on low-level modules. Both should depend on abstractions.
> Abstractions should not depend on details. 
> Details (concrete implementations) should depend on abstractions.

"低内聚"会引发"霰弹式修改" (Shotgun Surgery), 即一点小的改动, 导致不同的类须同时修改. "高耦合"则会引发"发散式变化" (Divergent Change), 即一个类因为不同的原因在不同方向上做出改变. 

### 团队一致性

1. 制定代码规范 (code style)
2. 符合英语规范: 函数用动词, 类名用名词, 符合英文语法
3. 建立词汇表, 尤其是项目涉及领域的动名词含义进行统一
4. 编程语言的标准应统一, 尝试较新的语言特性, 理解其成因.
5. 代码评审, 结对编程 (一人写代码, 一人同时审查)



