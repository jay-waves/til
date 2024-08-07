MySQL 是一种**开源的**数据库管理系统, DBMS. 支持并发使用, 和数据恢复.

MySQL, Oracle 和 Microsoft SQL Server 都是基于**客户机-服务器**的.

***

这里主要阐述和 Oracle 的区别:

Oracle 中可用 `Disable Constraint` 暂时禁用外键约束, 使数据插入更方便. 但是Mysql 中没有约束功能, 只能先删除外键再添加回来.

## 约束关系差异

MySQL的InnoDB存储引擎支持外键约束, 语法类似Oracle:
```sql
contraint fk_name foreign key (sub_id) references sub(id)
```

`on delete cascade` 外键删除时, 会自动删除引用行.

## 数据类型差异

|                        | Oracle                        | MySQL                                    |
| ---------------------- | ----------------------------- | ---------------------------------------- |
| numeric                | `NUMBER(12, 0)`               | `DECIMAL(12, 0)`, `TINYINT, INT, BIGINT` |
| floating               | `BINARY_DOUBLE, BINARY_FLOAT` | `FLOAT, DOUBLE`                          |
| variable-length string | `VARCHAR2`                    | `VARCHAR`                                |
| fixed-length string    | `CHAR`, `LONG`                | `CHAR`, `TEXT`                           |
| date+time              | `DATE`, `TIMESTAMP`           | `DATETIME`, `TIMESTAMP`                  |
| time                   | `TIME`                        | `TIME`                                   |
| span of time           | `INTERVAL`                    |                                          |
| date                   |                               | `DATE`                                   |
| binary data            | `BLOB` (<4GB)                 | `BLOB`, `TINYBLOB`, `LONGBLOB`           |
| large string           | `CLOB`, `NCLOB`               | `TEXT`                                   |
| raw binary data        | `RAW`, `LONG RAW`             | `BLOB`                                         |

