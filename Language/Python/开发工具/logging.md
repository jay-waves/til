日志级别: `DEBUG` < `INFO` < `WARNING` < `ERROR` 

当日志级别设置为 `INFO` 时, 更低级别不会显示, 更高级别会显示.
- `DEBUG`: 详细诊断信息
- `INFO`: 用于输出程序正常执行的日志
- `WARNING`: 某些意外情况, 但不阻止程序继续运行. 默认级别
- `ERROR`: 更严重的问题, 程序可能不能正确运行

```python
import logging

# 配置日志
logging.basicConfig(
	filename = 'a.log',                                  # 设置输出文件 
    level=logging.DEBUG,                                 # 设置日志级别
    format='%(asctime)s - %(levelname)s - %(message)s',  # 设置输出格式
    handlers = [
	    logging.StreamHandler()
    ],
)

logging.debug('hello')
logging.info('hello')
logging.warning('hello')
logging.error('hello')
logging.critical('hello')

# 创建用户自定义日志对象
my_logger = logging.getLogger('my_logger')
my_logger.setLevel(logging.DEBUG)

# 创建自定义句柄, 用于定义日志输出行为
handler = logging.FileHadnler(...) # 控制输出位置, 如控制台, 文件, 网络等.
handler.setLevel(logging.EDBUG)
handler.setFormatter(logging.Formatter('....'))
my_logger.addHandler(handler)
```

### 输出格式化

可用的格式化变量:
- `%(asctime)s` 时间, `asc` 代表 ASCII, 默认格式为 `YYYY-MM-DD HH:MM:SS,sss`
- `%(crated)f` 时间, 指 Unix 时间戳
- `%(levelname)s`: 日志级别, 如 `INFO`
- `%(message)s`: 日志消息
- `%(filename)s, %(pathname)s, %(module)s, %(funcName)s, %(lineno)d`: 控制该日志输出的 Python 源文件, 以及路径, 模块名, 函数名, 行号
- `%(thread)d, %(threadName)s, %(process)d` 进程线程名

### 为什么日志没有输出?

#TroubleShooting 

1. 日志级别设置错误
2. 日志路径或输出方式设置错误
3. 使用默认日志对象, 导致日志输出配置被覆盖. (详见下述)

用户不手动创建日志对象时, logging 库的会默认创建一个根日志对象 logging.root, 并使用默认的 Handler, 可用 `logging.getLogger()` 获取. 
对于任意 logger 对象, `basicConfig()` 函数**只有在首次调用时生效**, 用于配置用户定义的 handlers. 对于根对象 logging.root 也是如此.

```python
root_logger = logging.getLogger()
fro h in rot_logger.handlers:
	root_logger.removeHandler(h)

def basicConfig(self, **args):
	if not self.handlers: # handlers 非空时, 立即返回, 不会重复配置
		return
	else:
		...
```

如果在用户调用 `basicConfig()` 之前, 如果已有模块不小心提前调用了 `logging.info()` 等日志输出, logging 库就会自动调用一次 `basicConfig()` 函数, 导致后续用户的配置失效. 解决办法是尽量不使用默认根日志对象 (虽然最方便), 并将 `logger` 对象的声明和配置紧凑在一起, 防止 `basicConfig()` 被隐式调用.