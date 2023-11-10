prompt = "If you tell us who you are, we can personalize the messages you see."
prompt += "\nWhat is your first name? "
# 注意在提示最后留一个空格, 好区分输入和提示

name = input( prompt )
print( f"\nHello, {name}!" )

# 但是input读入会自动转化为字符串, 即数字也会转化为字符, 后续不能继续当数字用
# 可以看到此时数字输入会用引号标注

