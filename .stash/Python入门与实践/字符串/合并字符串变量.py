first_name = 'ada'
last_name = 'lovelace'
full_name = f'{first_name} {last_name}'
print(full_name)

# f是format简写，即设置格式。这里意思是， 将花括号中的变量名替换为字符串格式

print(f'Hello, {full_name.title()}! ')

# 早版本用的是format（）方法
full_name2 = "{} {}".format( first_name, last_name )