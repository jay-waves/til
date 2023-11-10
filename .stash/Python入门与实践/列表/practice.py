from msilib import sequence

# 练习4-6至4-9
for value in range( 1, 21 ):
    print( value )

sequences = list( range( 1, 1_000_001) )
# for value in sequences:
#     print( value , end=" ")
print( 
    sum( sequences ) )

odds = list( 
    range( 1, 21, 2 )
)
for odd in odds:
    print( odd )

cube = [ value**3 for value in range( 1, 11 ) ]
print( cube )