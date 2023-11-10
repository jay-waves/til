### WARNING:
```ad-danger
vscode调试和配置环境复杂, 自定义项极多, 变量极多. 
以下仅能说在我的PC上如此, 不普适
```
1. 我的环境是clang编译器, 和winVSdebugger, 要调试程序必须使用Developer Command Prompt
2. 我在VScode上构建c环境, 和VS类似, 问题也和VS编程遇到的问题类似
3. 使用clang而非gcc, 是因为其更好用的代码静态提示, 很容易读懂
#### 终端调试:
1. 如果调试在终端进行, 那么EOF循环读入结束条件就会事项, 因为终端因为某种原因不能使用^z
- 目前替代方法为: 用freopen重定向流, 再用读入文本文件来模拟

#### VSCODE特殊编程要求:
##### 提交测评机前, 必须注释掉:
1. 调试io文件需要`include<windows.h>`
2. 注释掉`#define _CRT_SECURE_NO_WARNINGS`
	- 这是用来防止vs对老函数的安全报错的, 必须要放在首行才能生效
3. 调试IO函数时, 文件名必须使用**绝对路径** , 提交前需改掉
##### 调试需:
1. 必须使用DCP启动
3. 调试io文件需要`include<windows.h>`
4. VS的c头文件必须使用VS自带的

### debugger快捷键
1. 添加断点: f9, (内联加shift)
2. 运行: f5, +shift停止, +^ shift重启
3. 单步调试: f11, 用vs会跳转到库函数, +shift跳出
4. 单步跳过: f10, 
5. 添加监视变量: alt+s
6. 