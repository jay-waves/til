motorcycles = [ 'honda', 'yamaha', 'suzuki' ]
print( motorcycles )

# add
motorcycles.append( 'ducati' )
motorcycles.append( 'dudu' )
print( motorcycles )

motorcycles.insert( 0, 'ducati2' )
print( motorcycles )

# delete
del motorcycles[ 0 ]
print( motorcycles )

# pop相当于弹出栈顶元素，并允许使用一次。 但删除的列表队尾
print( motorcycles.pop() )
print( motorcycles )
# 用pop可以模拟后入先出， 但pop其实可以弹出多个位置
print( motorcycles.pop( 0 ) )
# 根据内容删除值
motorcycles.remove('suzuki')
print(motorcycles)
# remove也可以简单记录删除的值，需要提前记录
    # remove只删除第一个指定的值，相同的值需要使用循环
rem = 'yamaha'
motorcycles.remove( rem )
print( motorcycles )

# organize
# sort()永久排序
cars = [ 'bmw', 'audi', 'toyota', 'subaru' ]
cars.sort()
print( cars )

cars.sort( reverse=True )
print( cars )

# sorted()临时排序
print( sorted( cars ) )
print( cars )

# reverse反转列表元素， 不是重新排序只是颠倒顺序
print( cars.reverse )

# 确定长度, 从1开始计算元素个数
len(cars)

