# 除了根据顺序将实参和形参关联之外, 还可以直接根据名称传递实参
def describe_pet( animal_type, pet_name ) :
    '''显示宠物信息.'''
    print( f"\nI ahve a { animal_type }.")
    print( f"My { animal_type }'s name is { pet_name.title() }." )

describe_pet( animal_type= 'hamster', pet_name= 'harry' )
describe_pet( pet_name= 'harry', animal_type= 'hamster' )
