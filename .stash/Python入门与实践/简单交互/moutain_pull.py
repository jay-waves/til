responses = { }
polling_active = True


while polling_active :
    name = input( "\nWhat's your name? " )
    response = input( "What's mountain would you like to climb? " )
    responses[name] = response

    repeat = input( "Would you like to continue to poll? (yes/no) " )
    if repeat == 'no' :
        polling_active = False
    
print( "\n--- polling results---" )
for name, response in responses.items() :
    print( f"{name} would like to climb {response} " )
