## 软件安全分类

- 控制流安全: 控制流完整性 (Control FLow Integrity, CFI)-->CFG
- 信息流安全: 机密性, 完整性
- 内存安全: 内存泄漏(生命周期管理), 堆栈溢出保护, 空指针解引用(UAF), 双重释放
- 数据类型安全: 类型检查, 类型转换检查 (如不同时间格式), 
- 系统资源安全: 权限隔离, 资源控制, Profiling, Sandbox

## 软件分析与漏洞检测

|                   | Error exists   | No Error Exists |
| ----------------- | -------------- | --------------- |
| Error Reported    | True Positive  | False Positive  |
| No Error Reported | False Negative | True Negative                |

- Soundness: report all defects (no false negatives)
- Completeness: every reported defect is an actual defect (no flase positives)

### Static vs Dynamic Tradeoffs

- Coverage
	- generalize to addtional traces?
- Soundness
	- every actual data race it reported
- Completeness
	- all reported warnings are actually races
- Overhead
	- run-time slowdown
	- memory footprint
- Programmer Overhead