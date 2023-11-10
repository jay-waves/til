import re


requested_toppings = []

if requested_toppings:
    for requested_topping in requested_toppings :
        print( f'Adding {requested_topping}.' )
    print( '\nFinished making your pizza!' )
else:
    print( "Are you sure youwant a plain pizza?" )

available_toppings = ['mushrooms', 'olives', 'green peppers',
    'pepperoni', 'pineapple', 'extra cheese' ]

requested_toppings = ['musrooms', 'french fries', 'extra cheese' ]

for requested_topping in requested_toppings :
    if requested_topping in available_toppings :
        print( f'Adding {requested_topping}.' )
    else :
        print( f"Sorry , we don't have {requested_topping}.")

    print( '\n Finished making your pizza! ' )
