# user_profiel.py

def build_profile( first, last, ** user_info ) :
    '''创建字典,包括我们知道的有关用户的一切'''
    user_info[ 'first_name' ] = first
    user_info[ 'last_name' ] = last
    return user_info

user_profile = build_profile( 'albert', 'einstein',
    location= 'princeton',
    field= 'physics' )

print( user_profile )
print(sorted(user_profile))

# ** 双星号让python创建一个字典, 并将所有键值对放入.

# 常见形参名如: *args **kwargs

