file = 'F:\\notes\\PythonStudy\\python编程入门篇\\文件与异常\\writing.txt' # window下绝对路径要处理转义字符(反斜杠)\
# 否则例如\t, \n这样的空字符都会被误读
# linux下,macOS下用的是斜杠/,就不会存在这样的问题
with open( file, 'a' ) as wfile :
    wfile.write( "azhe\n" )
    wfile.write( 'emmm \n' )
    # open函数的模式包括: r只读模式(省略默认) , w写入模式(>), a追加模式(>>), 读写模式(r+). 千万小心w对内容的覆盖,建议使用a

# python只能将字符串写入, 数值需要用str转化为字符串