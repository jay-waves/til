# alien_no_points.py

alien_0 = { 'color' : 'green', 'speed' : 'slow' }
# print( alien_0['points'] )
# 这样会报错, 因为键不存在. 可以使用get()指定此时返回一个默认值, 从而避免类似错误.

point_value = alien_0.get( 'points', 'No point value assigned.' )
print( point_value )

# 如果get没有指定第二个参数, 且键不存在就会返回None
