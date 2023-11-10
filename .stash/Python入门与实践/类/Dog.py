class Dog :
    def __init_( self, name, age ) : # _init_用来创建(初始化)一个新实例, 后面跟着传入的实参
        self.name = name   # self是自动传入的实参, 关联向目前创建的实例(dahuang), 
        # 这告诉python我们现在是在将dahuang这个变量创建为'类'Dog的一个实例
        # 也让dahuang这个实例可以使用类中的属性和方法
        # 使用self可以使变量在整个class类定义中都可以使用, 这样可以通过实例访问变量叫做 '属性'
        self.age = age
    
    def sit( self ) :
        print( f'the dog, { self.name } will sit down !' )
    
    def roll_over( self ) :
        print( f"it's { self.age } olds, but still roll over everywhere" )


da_huang = Dog( 'dahuang', 3 )
print( da_huang.sit() )
print( da_huang.roll_over() )

'''
类是一种实例的抽象
实例是类的某个具象
方法是类具有的特征
特征为类与实例所共有
但实例仍保有自己的特点
'''
'''
这就像: 
大黄 是 狗
狗 是 类
大黄 是 实例
大黄会翻滚
翻滚是 类 的 一种方法
也就说翻滚是 狗类 的共性(特性)
但大黄长得黄, 并不是所有狗都这样'''


'''思考: 为什么要加一个self?? \^_^/  /~_~\ '-' >_< @_@ *_* -.-
书中解释是:
每个与实例相关联的方法调用都自动传入实参self. self是一个指向变量本身的引用, 使得变量能够引用类的属性和方法
有些笼统.
python中给变量赋值就可能改变它的类型, 这是用什么做到的呢?
是因为每个变量背后都附带了类的所有定义? (这个有可能, 因为python是脚本语言, 也就是从上往下执行)
还是说和c一样, 是靠编译器向上寻找?? (这个肯定不可能)
'''