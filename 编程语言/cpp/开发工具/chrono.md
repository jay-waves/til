---
code: <time>, <chrono>
---

标准库 `chrono` 来自于古希腊语 χρόνος, 意为时间.

## c time

`time()` 获取 [UCT 时间](../../../操作系统/sss/本地化.md).   
```c
time_t t = time(nullptr);

t // seconds since epoch time, like 180880623
ctime(&t); // Sun Nov 11 18:42:11 2025

tm* tm_ptr = localtime(&t); 

tm* tm_ptr = gmtime(&t); // UTC time
```

缺点:
- 不适合多线程, 因为有内部缓冲区.
- 只支持操作系统提供的"时区", 不支持时区间转换.
- 麻烦

## c++ chrono 

chrono c++11:
- clock (时钟)
	- `system_clock` 系统时钟, 类似 `c time()`. 和 `time_t` 可以相互类型转换.
	- `steady_clock` 
	- `high_resolution_clock`
- time point (时间点)
- duration (时长)
	- `ns = nanoseconds = duration<int64_t, nano>;`  ($10^{-9}$ s)
	- `ms` $10^{-3}$ s
	- `s` 1s
	- `h, min`
	- `year, month, day, weekday` (c++20)
- `std::literals::chrono_literals` (时间字面量)
- 常量: `Sunday, Tuesday, last, February` (c++20)

**time_point 是从 epoch (时间原点, 如 1970/01/01) 起始的 duration. time_point 还受 clock 种类影响.**

获取 `time_point`:
```cpp
auto now = sysem_clock::now()

system_clock::to_time_t(now) // --> time_t

now.time_since_epoch() / 1.0s // --> duration / 1.0s --> duration
// 依赖标准库的隐式转换, 可能损失时间精度
```

获取 `duration`:
```cpp
auto t1 = steady_clock::now()

auto t2 = steady_clock::now()

// duration/duration 后, 会返回一个纯数字, 用于输出.
(t2 - t1) / 1ns;
```

### sys_days

c++20 的日历易用性非常好:
- `2025y/3/10` 表示 2025 年 3 月 10 日. 类型为 `year_month_day`
- `2025y/March/10d`
- `10d/3/2025`
- `2025y/5/last` 五月的最后一天

上述所有类型都可以转化为 `sys_days`, 即以天为单位的时间单位.

```cpp
template <class Duration>
using sys_time = time_point<system_clock, Duration>;

using sys_days = sys_time<days>;
```

### timezone

c++20 的 `<chrono>` 支持了任意时区间转换. 
- `std::chrono::time_zone` 表示一个时区
- `std::chrono::zoned_time` 指带有时区的时间

UTC 转换为上海时区时间:
```cpp
auto now = system_clock::now();

auto tz = get_tzdb().locate_zone("Asia/Shanghai");
zoned_time zt{tz, now}; // 2025-11-16 21:06 CST
```

上海时区转化为 UTC:

```cpp
auto tz = get_tzdb().locate_zone("Asia/Shanghai");

local_time lt = local_days{2025y / November / 16} + 30min;

auto utc = tz->to_sys(lt);
```


## 参考

https://zhuanlan.zhihu.com/p/1892632711019073793