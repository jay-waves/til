def greet_users( names ) :
    '''向列表中的每位用户发出简单的问候'''
    for name in names :
        msg = f"Hello, { name.title() }"
        print( msg )

usernames = [ 'banana', 'message', 'abbreviation' ]
greet_users( usernames )
