def build_person( first_name, last_name, age= None ) : # 形参没被调用会变灰
    '''返回一个字典, 期中包含有关一个人的信息'''
    person = { 'first': first_name, 'last': last_name }
    if age:
        person['age']= age
    return person

musician = build_person( 'jimi', 'hendrix', 27 )
print( musician )

# 这个函数不仅接受简单的文本信息, 而且将其放在一个更合适的数据结构中
# None视为占位值, 在条件测试中视为False