import json

'''当有用户名数据时欢迎
无用户名数据时记录并欢迎'''

"""file = 'username.json'

try :
    with open( file ) as f :
        username = json.load( f )
except FileNotFoundError :
    username = input( "What's your name ?" )
    with open( 'python编程入门篇\\文件与异常\\'+ file, 'w' ) as f :
        json.dump( username, f )
    print( f"Welcome ! { username },we will remember you next time. ")
else :
    print( f"Hello, {username}, welcome back !" )"""


# 下面对这个功能进行重构: 

def get_stored_username() :
    """如果存储了用户名，就获取它。"""
    filename = 'username.json'
    try :
        with open( 'python编程入门篇\\文件与异常\\' + filename ) as f :
            username = json.load( f )
    except FileNotFoundError :
        return None 
    else :
        return username
     
     
def get_new_username() :
    username = input("What is your name? ")
    filename = 'username.json'
    with open('python编程入门篇\\文件与异常\\' + filename, 'w') as f :
        json.dump(username, f)
        
        
def greet_user():
    """问候用户，并指出其名字。"""
    username = get_stored_username()
    if username:
        print(f"Welcome back, {username}!")
    else:
        get_new_username()
        print(f"We'll remember you when you come back, {username}!")

greet_user()