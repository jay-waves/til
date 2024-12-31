日志级别: `DEBUG` < `INFO` < `WARNING` < `ERROR` 

当日志级别设置为 `INFO` 时, 更低级别不会显示, 更高级别会显示.

```python
import logging

# 配置日志
logging.basicConfig(
	filename = 'a.log',                                 # 设置输出文件 
    level=logging.DEBUG,                                # 设置日志级别
    format='%(asctime)s - %(levelname)s - %(message)s'  # 设置输出格式
)

logging.debug('hello')
logging.info('hello')
logging.warning('hello')
logging.error('hello')
logging.critical('hello')
```