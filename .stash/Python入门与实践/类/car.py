"""一个可用于表示汽车的类。"""
class Car:
    """一次模拟汽车的简单尝试。"""
    def __init__(self, make, model, year):
        """初始化描述汽车的属性。"""
        self.make = make
        self.model = model
        self.year = year
        self.odometer_reading = 0
    def get_descriptive_name(self):
        """返回整洁的描述性名称。"""
        long_name = f"{self.year} {self.make} {self.model}"
        return long_name.title()
    def read_odometer(self):
        """打印一条消息，指出汽车的里程。"""
        print(f"This car has {self.odometer_reading} miles on it.")
    def update_odometer(self, mileage):
        """
        将里程表读数设置为指定的值。
        拒绝将里程表往回调。
        """
        if mileage >= self.odometer_reading:
            self.odometer_reading = mileage
        else:
            print("You can't roll back an odometer!")
    def increment_odometer(self, miles):
        """将里程表读数增加指定的量。"""
        self.odometer_reading += miles


class Battery:
        """一次模拟电动汽车电瓶的简单尝试。"""
    def __init__(self, battery_size=75):
        """初始化电瓶的属性。"""
        self.battery_size = battery_size
    def describe_battery(self):
        """打印一条描述电瓶容量的消息。"""
        print(f"This car has a {self.battery_size}-kWh battery.")
    def get_range(self):
        """打印一条描述电瓶续航里程的消息。"""
        if self.battery_size == 75:
            range = 260
        elif self.battery_size == 100:
            range = 315
            print(f"This car can go about {range} miles on a full charge.")
class ElectricCar(Car):
    """模拟电动汽车的独特之处。"""
    def __init__(self, make, model, year):
        """
        初始化父类的属性。
        再初始化电动汽车特有的属性。
        """
        super().__init__(make, model, year)
        self.battery = Battery()

'''开始在模块中存放类后, 具体的程序文档就不应该用类来命名了, 而应该用更具体地名字
比如存放类地模块叫car.py, 那么具体程序就应该叫my_car.py'''