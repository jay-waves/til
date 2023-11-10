# for 循环是一种遍历列表的有效方式，但不应在for 循环中修改列表，否则将导致Python难以跟踪其中的元素。
# 要在遍历列表的同时对其进行修改，可使用while 循环。
# 通过将while 循环同列表和字典结合起来使用，可收集、存储并组织大量输入，供以后查看和显示。

unconfirmed_users = [ 'yjw', 'phb', 'sxc', 'lxy' ]
confirmed_users = [ ]

while unconfirmed_users :
    current_user = unconfirmed_users.pop()
    print( f"Verifing user: { current_user.title() }" )
    confirmed_users.append( current_user )

print( "\nThe following users have been confirmed: " )
for confirmed_user in confirmed_users :
    print( '\t' + confirmed_user.title() )

