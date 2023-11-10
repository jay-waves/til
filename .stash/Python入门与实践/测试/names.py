from name_function import get_fomatted_name

print("(entre 'q' to quit at any time.)")
while True :
    first = input('please input first name: ')
    if first == 'q' :
        break
    
    last = input('please input last name: ')
    if last == 'q' :
        break
    
    formatted_name = get_fomatted_name( first, last )
    print( formatted_name )
    