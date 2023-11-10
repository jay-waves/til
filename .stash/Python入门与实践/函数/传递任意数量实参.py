def make_pizza( *toppings ) :
    '''打印顾客点的所有配料'''
    print( toppings )

make_pizza( 'peperoni' )
make_pizza( 'mushrooms', 'greem peppers', 'extra cheese' )

# *让python创建一个名为toppings的空元组, 并且所有值都封装到这个元组中

def make_pizza2( * toppings ) :
    print( "\nMaking a pizza with the following toppings: " )
    for topping in toppings :
        print( f" - { topping }" )

# 注意, 混用关键词实参/位置实参/任意数量实参时, python会先匹配
# 位置实参和关键字实参, 再将余下的实参都收集到最后的形参中

