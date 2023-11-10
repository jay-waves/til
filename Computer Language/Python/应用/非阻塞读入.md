 流密码到底怎么读入一个很长的无换行符的流？

```python
import threading

class KeyboardThread(threading.Thread):

    def __init__(self, input_cbk = None, name='keyboard-input-thread'):
        self.input_cbk = input_cbk
        super(KeyboardThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            self.input_cbk(input()) #waits to get input + Return

showcounter = 0 #something to demonstrate the change

def my_callback(inp):
    #evaluate the keyboard input
    print('You Entered:', inp, ' Counter is at:', showcounter)

#start the Keyboard thread
kthread = KeyboardThread(my_callback)

while True:
    #the normal program executes without blocking. here just counting up
    showcounter += 1

```

不使用子线程：
```python
import select
import sys


class KeyboardReader:
    def __init__(self, callback):
        self.callback = callback

    def read_input(self):
        # 通过select监控标准输入是否可读
        rlist, _, _ = select
        if rlist:
            # 读取输入并调用回调函数进行处理
            input_bytes = sys.stdin.buffer.read(1)
            self.callback(input_bytes)


def my_callback(inp):
    # 对读入的字节进行处理
    print(inp)


# 创建KeyboardReader对象并设置回调函数
reader = KeyboardReader(my_callback)

while True:
    # 读取标准输入是否可读，如果有输入，则读取输入
    reader.read_input()

    # 执行其他任务
```