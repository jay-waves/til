
## Schema ALTER

更新表结构:
```sql
-- 添加列
ALTER TABLE students ADD phone CHAR(20);

-- 删除列
ALTER TABLE students DROP COLUMN phone;

--- 定义外键, MySQL
ALTER TABLE orders 
ADD CONSTRAINT fk_orders_customers 
FOREIGN KEY (cust_id) REFERENCES customers (cust_id);
```

删除表:
```sql
DROP TABLE students;
```

重命名表:
```sql
RENAME TABLE students TO students2;
```

## INSERT

`insert` 插入行. 对于被频繁访问的数据库, 修改数据是很耗时的, 因为需要更新很多索引, 可使用 `insert low_priority into` 降低执行优先级.

```sql
/* 依赖于默认列顺序: */
insert into students values (null, 'yujiawei', null); -- id, name, math

/* 指定赋值顺序: */
insert into studets(name, id, math) values ('yujiawei', null, null); 
-- 该方法可以省略一些列
```

将检索的数据插入数据库中: `insert` + `select`. 注意, 从其他表导入数据时, 主键不能和现有的重复.
```sql
insert into students2(id, name, math) select id, name, math from students
```

### UPDATE 

`update` 和 `delete` 关键词要谨慎使用, 容易毁坏数据完整性约束.

```sql
update students
set email = 'yjw@email.com', math=99
where id = 1;
```

### DELETE

```sql
delete from students where id=1;
```

