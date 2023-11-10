# for 是对集合中每个元素都执行一个代码块
# while是循环, 直到条件不满足为止

prompt = "\nTell me something , and I will repeat it back to you :"
prompt += "\nEnter 'quit' to end the program. "
message = ""
while message != 'quit' :
    message = input( prompt )
    if message != 'quit' :
        print( message )
