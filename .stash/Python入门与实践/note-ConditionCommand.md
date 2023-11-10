## 条件语句
### 格式
```python
if 条件 :
    #do something
elif 条件2 :
    #also do something
else:
    #do something else
```
python不支持switch, 但可以用elif实现

### 多条件情况
1. 或or
2. 与and
3. and和or优先级低于> <等判断符号

## 循环语句
### while语句
实例:
```python
numbers = [ 12, 35, 46, 37, 44 ,2]
odd = []
even = []
while len( numbers ) > 0 :
    number = numbers.pop()
    if number % 2 == 0 :
        even.append( number )
    else :
        odd.append( number)
```
* 无限循环

```python
while 1 :
    print( hello )
    break 
    #continue
```

无限循环中,你可以摁^c跳出循环

* while, else语句

```python
while 1 :
    # do somthing
else : #如果while条件不成立, 就执行else
```
其实就相当于正常执行完循环后, 再执行一段程序

### For语句
**用来遍历任何有序列的项目, 比如一个列表或一个字符串**
```python
# e.g.
str = 'python'
for letter in str :
    print( letter +　＇　＇ )
# result: p y t h o n 这里是letter依次赋值为组中的每一个元素, 并执行一次循环.

fruits = [ 'banana', 'apple', 'mango']
for fruit in fruits :
    print( fruit )
# result: bananaapplemango

#遍历字典会遍历标识符
#如果想遍历字典返回值, 需要用for i in dict.items()
```
#### Range函数
用来控制循环次数的函数, 可以用来遍历数字序列.
```python
>>>range(5) #直接生成序列
0,1,2,3,4
>>>range(5,9) #规定起始, 注意包括头不包括尾; 和截取的逻辑一样
5,6,7,8
>>>range( 0, 10, 3) #规定步长, 步长可以时负数
0,3,6,9
``` 
* **结合range()和len()以遍历一个序列的索引**
如
```python
    for i in range( len( a ) ) 
    # len(a) 返回序列的元素个数
```
* 也可以用来快速创建一个列表
```python
>>>list( range(5) )
[0, 1, 2, 3, 4]
```
#### 使用enumerate()遍历
```python
>>>sequence = [ 12, 34, 34, 23, 45, 76, 89 ]
>>>for i, j in enumerate(sequence): #枚举
        print( i, j )
...
...
0 12
1 34
2 34
3 23
4 45 
5 76
6 89
```

### python推导式
从一个数据序列直接构建另一个数据序列的结构
* 列表推导式:
```python
>>> names = ['Bob','Tom','alice','Jerry','Wendy','Smith']
>>> new_names = [name.upper()for name in names if len(name)>3]
>>> print(new_names)

#output = ['ALICE', 'JERRY', 'WENDY', 'SMITH']
```
结构为: *表达式* for *变量* in *序列* ( if *要求序列中变量满足的条件* )

* 字典推导式:
```python
listdemo = ['Google','Runoob', 'Taobao']
# 将列表中各字符串值为键，各字符串的长度为值，组成键值对
>>> newdict = {key:len(key) for key in listdemo}
>>> newdict
{'Google': 6, 'Runoob': 6, 'Taobao': 6}
```

* 集合推导式
```python
>>> setnew = {i**2 for i in (1,2,3)}
>>> setnew
{1, 4, 9}
```