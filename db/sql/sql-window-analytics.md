
窗口函数 `OVER` 不会聚合合并行，而 `GROUP` 会聚合并减少行数。
可以理解为，窗口函数用一个小范围内的窗口进行滑动与计算。

`PARTITION BY` 用于行分组，当没有分组时，所有行属于同一组，不重置窗口函数。
`QUALIFY` 则专用于对每组的内部行进行二次筛选。

```sql
foo() OVER (
    PARTITION BY ... [, ...]
    ORDER BY ....
    ROWS BETWEEN ... AND ...
)
[QUALIFY ...]
```

`foo()` 是指窗口函数，有以下四类：
- 排名函数 `ROW_NUMBFER()`, `RANK()`
- 偏移函数 `LAG()` 上一行，`LEAD()` 前一行
- 普通聚合函数 `SUM(), COUNT()`
- 分布比例函数 `NTILE(), PERCENT_RANK(), CUME_DIST()`


```sql
SELECT
    customer,
    month,
    amount,
    LAG(amount, 1) OVER (
        PARTITION BY customer
        ORDER BY month
    ) AS prev_amount
FROM orders;
```

计算最近三行的滚动销售额：

```sql
SUM(amouont) OVER (
    PATITION BY customer
    ORDER BY month
    ROWS BETWEEN 2 PRECEDING and CURRENT ROW 
)
```

**注意，窗口函数的执行顺序相对靠后**：

FROM/JOIN  >  WHERE  >  GROUP BY ... HAVING  >  Window Function  >  SELECT  >  DISTINCT  >  ORDER BY
