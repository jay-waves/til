## `where`

使用 `where` 条件过滤数据.

匹配字符串: 注意, mysql 默认不区分大小写.

```sql
select grades from students where name = 'yjw';
```

不匹配检查:

```sql
select name from students where id <> 1003;
```

范围检查:

```sql
select name from students where math between 80 and 100;
```

空值: `null` 

```sql
select name from students where math is null;
```

逻辑组合:

```sql
select name from students where math > 80 and chinese > 80;

select name from students where math<60 or chinese<60;
```

包含: `in` 操作符后, 常跟随子句.

```sql
select name, grades from students where id not in (1003, 1004);
```

注意, mysql 仅允许 `not` 对 `in`, `exists`, `between` 取反.

### 通配符

`like` 是 sql 的一种谓词, 指示使用通配符进行比较. 通配符 (wildcard) 可能降低效率, 尤其不应放在匹配开始处.

- `%` 表示任意字符任意次数, 但不匹配 `null`: 

```sql
select name from students where name like `%Yu%`
```

- `_` 匹配单个字符.

### [正则表达式](../../appx/正则表达式表.md)

正则表达式匹配: `regexp`. 区分大小写的正则表达式：`regexp binary`


```sql
select name from students where name regexp '\\([a-z] sticks?\\)'
```

`sticks?` 匹配 sticks 或 stick

## Order By & Group By

```sql
SELECT name, id FROM students ORDER BY id;
```

可以按多个列排序, 即稳定排序:

```sql
SELECT name, id FROM students ORDER  BY math, english;
```

降序:

```sql
SELECT name, id FROM students ORDER  BY math DESC, english;
```

`DESC` 只作用于 math 列, 定语应后置. `ASC` 指升序, 但升序是默认的.

找出最大值:
```sql
SELECT name, math FROM students ORDER BY math DESC LIMIT 1;
```

分组: 
```sql
select name, id,  math from  students group by id;
```

## Join (多表联合查询)

![|500](http://oss.jay-waves.cn/til/数据库_JOIN语法.avif)

套集合关系:

|     | =   | <>     | <    | <=    | >    | >=    |
| --- | --- | ------ | ---- | ----- | ---- | ----- |
| ANY | IN  | --     | <MAX | <=MAX | >MIN | >=MIN |
| ALL | --  | NOT IN | <MIN | <=MIN | >MAX | >=MAX      |


```sql
SELECT vend_name, prod_name, prod_price FROM vendors, products
WHERE vendors.vend_id = products.vend_id
ORDER BY vend_name, prod_name;
```

使用表名来完全限定列名.`vend_id` 是 `vendors` 表的主键, `products`表的外键.

使用 `where` 创建连结, 因为 SQL 实际的工作是将两表的每一行进行匹配. 如果不指定条件, 输出是两表的笛卡尔积, 即所有组合的可能.

上述条件是**相等**, 所以也称为等值连结或内部连结, SQL 对该类型支持一种推荐语法:
```sql
SELECT vend_name, prod_name, prod_price 
FROM vendros INNER JOIN products
ON vendors.vend_id=products.vend_id;
```

上述 `inner join` 指**内部联结**, 若 `vendors` 没有匹配的 `products`, 就不会被打印. 使用**外部联结** `outer join`, 可以打印无匹配的行, 值为 `NULL`. `left outer join` 会打印所有 `vendors`, 如果没有对应 `products` 就会显示 `null`; `right join` 反之, 显示右表的所有行, 左表如果没有匹配, 则显示null.

以笛卡尔积角度理解, 内部联结忽略结果零值, 外部联结会显示为 `null`.

### 多表联结:

连结: 货物表+订单表+供货商表
```sql
SELECT prod_name, vend_name, prod_price, quantity
FROM orderitems, products, vendors
WHERE products.vend_id = vendors.vend_id
	AND orderitems.prod_id = products.prod_id
	AND order_num = 20005;
```

### 自联结

常用来替代子语句, 速度更快.
```sql
SELECT p1.prod_id, p1.prod_name 
FROM products AS p1, products AS p2
WHERE p1.vend_id = p2.vend_id AND p2.prod_id = 'hello';
```

### 全称量词 (For All)

数据库没有全称量词 (for all), 需要用存在量词等价转换: $(\forall x) P\equiv \neg (\exists x(\neg P))$, 其中量词 $x\in Q$, P 为谓词. 举例而言, "选择了所有课程的学生" 等价于 "不存在有课程没有被该学生选过", 其中 $x=\text{课程}\in Q=\text{所有课程集合}$, $P=\text{学生选择了该门课}$.

## Aggregation

注意 `where` 不能使用聚合函数, 需要使用 `having`. 除 `count()` 外的所有聚合函数会忽略 `null` 行, 分组时多个 `null` 会被分为一组.

- `count()`
- `sum()`
- `avg()`
- `max()`
- `min()`

```sql
select CustomerID, SUM(Amount) as TotalAmount
from Orders
group by CustomerID
having SUM(Amount) > 500;
```

## Union

将多个 `select` 语句结果组合为一次查询结果. 在同时从多个表中查询类似数据时, 比 `where` 复合条件简洁.

```sql
select vend_id, prod_id, prod_price from products where prod_price <= 5
UNION -- 组合关键字
select vend_id, prod_id, prod_price from products where vend_id in (1001, 1002);

/* 等价于 */

select vend_id, prod_id, prod_price from products
where prod_price <= 5 OR vend_id in (1001, 1002);
```

`union` 会去掉重复, `unioin all` 则会全部显示. 个别数据库还支持 `minus` 和 `intersect`.

