banned_list = []

import help_functions
banned_list.append((5,2))

if (3,2) not in banned_list:
        print("lol")



def neighbours_ban(ban_list, range_var, x_ban, y_ban):
    for i in range(range_var):
        for j in range(range_var):
            ban_list.append((x_ban+i - 1,y_ban+j))
            ban_list.append((x_ban+i,y_ban+j - 1))
            ban_list.append((x_ban+i - 1, y_ban+j - 1))
            ban_list.append((x_ban+i + 1, y_ban+j))
            ban_list.append((x_ban+i,y_ban+j + 1))
            ban_list.append((x_ban+i + 1, y_ban+j + 1))
            ban_list.append((x_ban+i + 1, y_ban+j - 1))
            ban_list.append((x_ban+i - 1,y_ban+j + 1))
    return ban_list


banned_list = neighbours_ban(banned_list,10,5,2)

print(banned_list)
print(banned_list[0])
x, y = banned_list[0]
print(x,y)

if (4,2) not in banned_list:
        print("LOL")

a,b,c,d,e,f = help_functions.getData()