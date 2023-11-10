# 遍历键值对
user_0 = {
    'username' :  'efermi',
    'first' : 'enrico',
    'last' : 'fermi', 
}

for key, value in user_0.items():
    print( f"\nKey: {key}" )
    print( f"Value: {value}" )
# key和value两个变量分别储存键和值, 这俩是随便指定的
# 可以看出每个for循环如果是多行输出, 就会自动空行.

for key in user_0.keys() :
    print( key.title() )

# 按特定顺序遍历字典
favorite_languages = {
    'jen' : 'python', 
    'sarah' : 'c', 
    'edward' : 'ruby', 
    'phil' : 'python',
}
for name in sorted( favorite_languages.keys() ) :
    print( f"{name.title()}, thank you for taking the poll." )

for value in user_0.values() :
    print( value.upper() )
# 考虑遍历内容时去重: 使用 集合
# 因为set不允许重复元素

print( 'the following languages have been mentioned: ' )
for language in set( favorite_languages.values() ) :
    print( language.title() )



