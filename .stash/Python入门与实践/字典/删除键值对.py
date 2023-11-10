alien_0 = { 'color' : 'green', 'points' : 5 }
print( alien_0 )

del alien_0['points']
print( alien_0 )
# 此操作不可恢复

# 这几个文档演示了一个字典储存一个对象的多种信息
# 但还可用一个字典储存多个对象的类似信息:
favorite_languages = {
    'jen' : 'python',
    'sarah' : 'c',
    'edward' : 'ruby',
    'phil' : 'python', # 再最后一个键值对后也添加一个逗号, 方便下一行编辑
    }   
