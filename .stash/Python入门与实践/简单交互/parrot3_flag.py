# for 是对集合中每个元素都执行一个代码块
# while是循环, 直到条件不满足为止

prompt = "\nTell me something , and I will repeat it back to you :"
prompt += "\nEnter 'quit' to end the program. "
active = True # This is the flag

while active :
    message = input( prompt )
    if message == 'quit' :
        active = False
    else :
        print( message )
    
# 用一个flag作为统一的while判断条件在条件众多时有显著作用
# 这样, 一方面条件清晰, 另一方面可以将众多条件判断隐藏在其他地方( 隐藏细节 )

