age_1 = 22
age_2 = 18 
if age_1 >= 21 and age_2 >= 21 :
    print( 1 )
# readability
if (age_1 >= 21) and (age_2 >= 21):
    print( 1 )
else :
    print( 0 )

if (age_2 >= 21) or (age_1 >= 21) :
    print( 1 )

# if -> elif -> else
age = 12

if age < 4:
    price = 0
elif age < 18 :
    price = 25
elif age < 65 :
    price = 40
else:
    price = 20

print ( f'Your admission cost is {price}' )


    
