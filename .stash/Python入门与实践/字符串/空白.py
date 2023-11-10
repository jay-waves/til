favorite_language = 'python '
print( favorite_language.rstrip() + 'hello' )
print( favorite_language + 'hello' )
# \t是制表符, 会缩进
#处理多余空白, 避免误会, 用rstrip方法暂时删除结尾空白

#永久删除，必须将删除操作的结果关联到变量
favorite_language = favorite_language.rstrip()
print( favorite_language + 'hello' )


# 前空白用 favorite_language.lstrip()
# 两边空白 favorite_language.strip()

