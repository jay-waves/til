# 给每个形参指定默认值
def describe_pet( pet_name, animal_type= 'dog' ) :
    '''显示宠物信息.'''
    print( f"\nI ahve a { animal_type }.")
    print( f"My { animal_type }'s name is { pet_name.title() }." )

describe_pet( pet_name= 'willie' )
# 注意函数形参顺序发生了变化. 
# 使用默认值时, 必须先列出没有默认值的形参, 再列出有默认值的实参
# 这让python依然能够正确解读位置实参

describe_pet( 'whillie' )

