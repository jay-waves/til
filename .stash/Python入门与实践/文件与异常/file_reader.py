with open('python编程入门篇\文件与异常\pi_digits.txt') as file_object:
    contents = file_object.read()
print(contents)
# 缺点是末尾会比文件本身多一个空行，这是因为读取文档末尾时会返回一个空字符串
print( contents.rstrip() )
# 这样可以解决多余空行问题

# 注意，不是一个文件夹内就可以引用。要看你终端的相对位置在哪。
# 比如终端位置也在该文件夹就可以应用， 所以最好还是用绝对位置

file = 'python编程入门篇\文件与异常\pi_digits.txt'

with open( file ) as file_object :
    for line in file_object :
         # print( line ) 这样会发现每次循环都多一个空行，这是因为txt中每行都有一个换行符，而print本身也有一个换行符
        print( line.rstrip() )

with open( file ) as file_object:
    lines = file_object.readlines() # 逐行读入数据，每个元素对应一行数据
for line in lines:
    print(line.rstrip())
# 这样的好处是lines这个列表可以在整个文档中用。 但with打开的文档只能在with代码块中用

pi_spring = ''
for line in lines :
    pi_spring += line.rstrip()

print( pi_spring )
