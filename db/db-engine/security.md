

## 数据安全

安全性控制方法:
- **用户标识** (Identification) 和 用户鉴别 (Authentication), I&A
- [**存取控制** (Access Control)](../../sec/security-models.md): 用户权限定义 + 合法权限审查
- 视图 (View)
- [审计 (Audit)](../../sec/security-models.md)
- 数据加密 (Encryption)
- 数据备份与恢复

### SQL-DAC

SQL 语句中定义 `GRANT` 和 `REVOKE` 语句来实现自主存取控制 (DAC), 定义权限的过程称为"授权":

SQL 

```sql
revoke <权限1> [, <其他权限> ...]
on <对象类型> <对象名>[, <对象类型> <对象名> ...]
from <用户> [, <用户>...] [cascade | restrict]

grant <权限1> [, <其他权限> ...]
on <对象类型> <对象名>[, <对象类型> <对象名> ...]
to <用户> [, <用户>...]
[with grant option] -- 允许其权限传播
```

举例:

```sql
grant update(no), select, insert(name)
on table students
to teacher;
```

