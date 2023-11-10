# 优秀的处理错误可以避免无意的用户失误和恶意攻击
# try中放可能产生错误的代码
# except中表现如果出现错误要干什么
# else中放一些try成功后需要执行的代码

'''division-calculator'''
print( 'you can input two numbers, and i will divide them !' )
print( 'enter q at any time to quit)' )

while True :
    first_num = input( 'the first number is: ' ) 
    if first_num == 'q' :
        break
    sec_num = input( 'the second number is: ' ) 
    if sec_num == 'q' :
        break

    try :
        answer = int( first_num ) / int( sec_num )
    except ZeroDivisionError :
        print( "you can't devide a number by zero !!" )
    else :
        print( answer )
    # try-except-else代码块