各种容器的抽象基类

| ABC           | Inherits From                | Abstract Methods                     | Mixin Methods                                                                         |
| ------------- | ---------------------------- | ------------------------------------ | ------------------------------------------------------------------------------------- |
| **Container**   |                              | `__contains__`                       |                                                                                       |
| Hashable    |                              | `__hash__`                           |                                                                                       |
| **Iterable**    |                              | `__iter__`                           |                                                                                       |
| **Iterator**    | Iterable                     | `__next__`                           | `__iter__`                                                                            |
| Reversible  | Iterable                   | `__reversed__`                       |                                                                                       |
| **Generator**   | Iterable                   | `send, throw`                        | `close`, `__iter__`, `__next__`                                                           |
| **Sized**       |                              | `__len__`                            |                                                                                       |
| **Callable**    |                              | `__call__`                           |                                                                                       |
| Collection  | Sized, Iterable, Container | `__contains__, __iter__, __len__`    |                                                                                       |
| Sequence    | Reversible, Collection     | `__getitem__, __len__`               | `__contains__`, `__iter__`, `__reversed__`, `index, count`                                  |
| ByteString  | Sequence                   | `__getitem__, __len__`               |                                                                                       |
| Set         | Collection                 | Collection's Methods    | `__le__, __lt__, __eq__, __ne__, __gt__, __ge__`, `__and__, __or__, __sub__, __xor__` |
| MutableSet  | Set                        | Collection's Methods, `add`, `discard` | Set's Mixin Methods, `clear`, `pop`, `remove`, `__ior__`...                                   |
| Mapping     | Collection                 | `__getitem__`, `__iter__`, `__len__` | `__contains__`, `keys`, `items`, `values`, `get`, `__eq__`, `__ne__`                  |
| `MappingView` | `Sized`                      |                                      | `__len__`                                                                             |
| `ItemsView`   | `MappingView`, `Set`         |                                      | `__contains__`, `__iter__`                                                            |
| `KeysView`    | `MappingView`, `Set`         |                                      | `__contains__`, `__iter__`                                                                                      |

自行实现所有*协议*可能会有遗漏, `collections.abc` 提供了模板.

```python
from collections.abc import Sequence

class BadType(Sequence):
	pass

foo = BadType()
>>> TypeError: can't instantiate abstract class with abstract methods __getitem__, __len__, ...
```

也可以直接继承内置类型:

```python
class FrequencyList(list):
	def __init__(self, members):
		super().__init__(members)
	def frequency(self):
		counts = {}
		for item in self:
			counts.setdefault(item, 0)
			counts[item] += 1
		return counts
```
