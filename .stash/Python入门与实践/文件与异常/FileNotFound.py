file = 'hello.txt'
try :
    with open( file, encoding= 'utf-8' ) as f :
        contents = f.read() 
except FileNotFoundError :
    print( f'sorry, the file { file } is not found' )
else :
    words = contents.split() # split方法将以空格分隔符为单位将字符串拆分为列表元素. 可用于统计字数
    print( f"the file {file} contains { len(words) } words. " )

