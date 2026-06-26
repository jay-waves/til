## 事务

事务 (Transaction) 是用户定义的一系列数据库操作, 有原子性(不可分割), 不等于程序. **事务是恢复和并发控制的基本单位**.

```sql
begin transaction
	sql1
	sql2
commit    -- 事务正常结束, 提交事务所有操作(写入)

begin transaction
	sql1
	sql2
rollback  -- 事务异常终止, 回滚到开始状态
```

### 事务 ACID 特性

ACID 特性:
- 原子性, atomicity
- 一致性, consisitency
- 隔离性, isolation
- 持续性, durability

## 故障

- **事务内部(非预期)故障** --> 事务撤销, 借助日志.
- **系统故障**, 造成系统停止运转, 必须重启 --> 重做所有已提交事务 (因为可能尚未写入), 并强制撤销所有未完成事务, 借助日志.
- **介质故障**, 指外存故障
- 计算机病毒

## 恢复

利用**冗余**数据来重建数据库, 冗余数据包括:
- 数据转储 (backup), 备份
- 登记日志文件 (logging). + 建立检查点 (checkpoint)

### 数据转储

动态转储与静态转储, 海量转储与增量转储

### 日志文件

```
                静态转储       运行事务   
正常运行     --|----------|----------------|-----
             Ta         Tb                Tf (故障发生点)
                        登记日志       
                        +-------------

             重装后副本    用日志文件恢复事务    继续运行
介质故障恢复  -----------|- --- --- --- --- -|--------->
                                           登记日志
                                           +--------->
```

登记日志原则:
- 登记次序严格按照并发事务执行的时间次序
- 先写日志文件, 后写数据库


### 事务处理 Transaction

当自动化执行一组 sql 指令时, 如果出现错误而指令执行不完整, 此时可使用 rollback 自动回退到执行前状态.

- 事务 transaction, 指一组 sql 语句.
- 回退 rollback, 撤销指定 sql 语句的过程.
- 提交 commit, 将未存储的 sql 语句结果写入表.
- 保留点 savepoint, 使用它发布部分回退, 而不是整个事务.

应使用支持事务管理的引擎, 如 InnoDB, 并且事务管理只支持处理: `delete`, `update`, `insert` 语句.

```sql
-- 回滚
start transaction;
/* do sth */
rollback;

-- 提交, 先修改虚拟表, 当所有语句成功执行后, 才真正将修改结果同步到数据库. 非事务管理中, 提交都是隐式立即进行的.
start transaction;
/* do sth */
commit;

-- 创建保留点
start transaction;
/* ... */
savepoint hello1;
/* ... */
rollback to hello1;

-- 更改默认提交行为, 由内建布尔值 autocommit 控制. 仅针对此连接.
set autocommit=0; 
```