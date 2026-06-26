注意，Maple Tree 只支持非交叠的范围。

|                      | [Red Black Tree](../../algo/tree/red-black-tree.md)   | [Radix Tree](../../algo/tree/xarray.md) | Maple Tree       |
|----------------------|----------|------------|------------------|
| RCU Safe             | No       | Yes        | Yes              |
| Range support        | Yes      | Limited    | Non-overlapping  |
| Tree height          | Tall     | Short*     | Medium           |
| API                  | Hard     | Easy       | Easy             |
| Node                 | Embedded | External   | External         |
| Node size            | 24 bytes | 576 bytes  | 128 bytes        |


#TODO 