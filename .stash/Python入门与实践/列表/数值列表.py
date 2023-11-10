squares = []
for value in range( 1, 11 ): # 注意value不是临时变量
    square = value ** 2
    squares.append( square )

print( squares )

print( min(squares) )
print( max(squares) )
print( sum(squares) )

# 列表解析
squares = [ value**3  for value in range( 1, 11 ) ]
print( squares )
