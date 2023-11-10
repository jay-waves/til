# 使用切片复制复制列表:
my_foods = ['pizza', 'falafel', 'carrot cake' ]
friend_foods = my_foods[ : ]

print( f"My favorite foods are: {my_foods}" )
print( f"\n My friend's favorite foods are: {friend_foods}")

# 注意赋值并不是产生副本，而只是添加了一个指针， 指向的还是相同的列表。
'''这会在哪里体现出差异呢？ 就是赋值后，你对一个列表进行操作， 另一个列表也会这样做
但有时, 我们只是想复制一下, 而不是完全同步'''