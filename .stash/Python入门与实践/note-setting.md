## python 交互程序

不同来开发代码, 主要用来简单调试代码, 代码一般只能一行一行执行. 多行需要\和^z等奇淫技巧.
* 中文编码:
    问题描述:
        python编码默认格式是ASCII格式, 无法正常打印格式
    解决办法:
        文件头加上
                # -*- coding: UTF-8 -*-
            or
                # coding=utf-8 (=左右不要加空格)
    后续:
        1. python3.0以后默认使用utf-8所以可以正常解析中文
        2. 编辑器要同时设置文件储存格式为utf-8
* 退出python解释器:
    exit() 或 ^Z;
* python -h获取各参数帮助信息

### 脚本式编程
* linux下可以在脚本顶部添加
`#! /usr/bin/env python3` 
来让python脚本像SHELL脚本一样可以直接执行
1. 然后修改脚本权限, 使其有执行权限 <br>
`$ chmod +x hello.py`
2. 执行命令: <br>
`$ ./hello.py`
