### tricks
- 延长仿真时间：
1. 修改默认值：process/property/simulaition-time
2. 在ISim的Console中输入命令，如`#run 1s`

- debugger查看变量
仿真后好像无法查看中间变量，目前比较好的方法是，将想查看的变量设置为一个输入output，一起放到时序仿真里去看。
如添加：
```verilog
module{
...,
output variable_debugger,//添加值
}

reg variable;//要查看的变量

assign variable_debugger = variable
```

### ERROR
- `this signal is connected to multiple drivers`
在不同进程中对同一寄存器进行了不同的修改，这时候程序不知道该采用哪个的赋值（竞争）。常见有在initial和always块中都对某变量进行赋值了，此时使用rst代替初始化是更好的方法，因为initial是只能仿真不能综合的。
解决办法：**一个变量只在至多一个模块中进行修改**，当然，可以在多个模块中read—only使用。

- `Part-select of vector reg array 'coeff' is illegal.`` 
如果提示`Illegal right hand side of blocking assignment`，一般是把需要用reg的地方用成wire了，常见比如把input的wire变量直接赋值给reg，或者想在组合逻辑里给wire赋值。。

如果提示`Illegal left hand side of continuous assignment`，一般是把需要用wire连续赋值的地方用成reg了。
