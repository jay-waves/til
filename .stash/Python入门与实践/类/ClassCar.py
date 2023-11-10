from inspect import Attribute


class Car :
    '''简单模拟汽车门类'''
    def __init__( self, name, model, year ) :
        self.name = name
        self.model = model
        self.year = year 
        self.odometer = 0
    
    def get_description( self ) :
        name = f"{ self.name } { self.model } { self.year }"
        return name.title()
    
    def read_odometer( self ) :
        print( f"The car has { self.odometer } miles on it " )

    def update_odometer( self, mileage ) :
        if mileage >= self.odometer :
            self.odometer = mileage 
        else :
            print( "error! you can't roll back the odometer" )

    def increment_odometer( self, mileage ) :
        self.odometer += mileage 


my_car = Car( 'audi', 'a4', '2021' )
print( my_car.get_description() ) 

my_car.update_odometer( 23_500 )
my_car.read_odometer() 

my_car.increment_odometer( 24 )
my_car.read_odometer()

class ElectricCar( Car ) :
    def __init__(self, name, model, year): # 初始化子类需要的参数
        super().__init__(name, model, year) # 父类也称为超类（SuperClass), super()方法使能够调用父类方法和属性
        # 赋值可以调用子类的_init_(), 子类—init（）会用super调用父类的init（），子类就会获得父类的属性
        self.battery = Battery() 
    
    def battery_info( self ) :
        print( f"This car has a { self.battery_size } battery" )


my_tesla = ElectricCar( 'tesla', 'model s', '2012' )
print( my_tesla.get_description() )
print( my_tesla.battery_info() )

# 有时类中也可包含类， 比如电动车是很大的概念， 你可能想将其中的电池拿出来单独作为一个类
class Battery :
    def __init__( self, battery_size ) :
        self.battery_size = battery_size
    
    def battery_info( self ) :
        print( f"the battery is { self.battery_size }-kWh " )
