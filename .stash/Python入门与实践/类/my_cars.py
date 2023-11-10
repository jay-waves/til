from car import Car, ElectricCar

my_new_car = Car('audi', 'a4', 2019)
print(my_new_car.get_descriptive_name())

my_new_car.odometer_reading = 23
my_new_car.read_odometer()

my_tesla = ElectricCar('tesla', 'roadster', 2019)
print(my_tesla.get_descriptive_name())

'''
两种全部导入方式:
1. from car import * 
    这样的好处是不需要句点表达法, 坏处是不清晰/易名称冲突
2. import car
    这样导入模块, 需要使用句点表达法
    好处是不会起冲突, 因为必须要加类名'''