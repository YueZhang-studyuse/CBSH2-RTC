
from hashlib import new
from importlib.resources import path


def readpath(num_of_agents):
    paths = {}
    for i in range(25):
        single_scen_path = []
        file_name = "mcp/" + str(num_of_agents) + "agents/" + str(num_of_agents) + "agentspaths" + str(i+1) + ".txt"
        f = open(file_name,"r")
        line = f.readline()
        while(line):
            temp = line.strip().replace("/n","").split(" ")[2].split("->")[0:-1]
            for j in range(len(temp)):
                loc = temp[j].replace("(","").replace(")","").split(",")
                loc[0] = int(loc[0])
                loc[1] = int(loc[1])
                temp[j] = loc
            single_scen_path.append(temp)
            line = f.readline()
        paths[int(i+1)] = single_scen_path
        f.close()
    return paths

def read_init_direction(num_of_agents,paths):
    for i in range(25):
        file_name = "mcp/directions/random-32-32-20-random-" + str(i+1) + ".scen"
        f = open(file_name,"r")
        line = f.readline()
        for j in range(num_of_agents):
            line = f.readline()
            temp = line.strip().replace("/n","").split("\t")
            paths[i+1][j][0].append(int(temp[6]))
            paths[i+1][j][-1].append(int(temp[9]))

def add_directions(single_path):
    index = 1
    while (index < len(single_path) - 1):
        previous_direction = single_path[index-1][2]
        x1 = single_path[index-1][0]
        y1 = single_path[index-1][1]
        x2 = single_path[index][0]
        y2 = single_path[index][1]

        if (x2-x1 == 1 and y2 == y1):
            single_path[index].append(2)
        if (x2-x1 == -1 and y2 == y1):
            single_path[index].append(0)
        if (x2 == x1 and y2-y1 == 1):
            single_path[index].append(1)
        if (x2 == x1 and y2-y1 == -1):
            single_path[index].append(3)
        if (x2 == x1 and y2 == y1):
            single_path[index].append(single_path[index-1][2])
        index += 1
        #index -= 1

def preprocess_orders(scen_path):
    loc_order = {}
    max_line = 0
    for path in scen_path:
        if (len(path)>max_line):
            max_line = len(path)
    for i in range(max_line):
        for agent in range(len(scen_path)):
            if (i >= len(scen_path[agent])):
                continue
            temp = str(scen_path[agent][i][0])+","+str(scen_path[agent][i][1])
            if temp in loc_order.keys():
                #skip an wait action
                if i > 0 and scen_path[agent][i-1][0] == scen_path[agent][i][0] and scen_path[agent][i-1][1] == scen_path[agent][i][1]:
                    continue
                loc_order[temp].append(agent)
            else:
                loc_order[temp] = [agent]
    return loc_order

def build_delay(scen_path):
    new_path = []
    # for item in scen_path:
    #     new_path.append([item[0]])
    for path in scen_path:
        single_path = []
        single_path.append(path[0])
        single_index = 1
        while (single_index < len(path)):
            curr_direction = path[single_index][2]
            pre_direction = path[single_index-1][2]
            #do not have to turn
            if (curr_direction == pre_direction):
                single_path.append(path[single_index])
            #need turn left first
            elif (curr_direction - pre_direction == -1 or (curr_direction == 3 and pre_direction == 0)):
                new_direction = pre_direction - 1
                if new_direction == -1:
                    new_direction = 3 
                single_path.append([path[single_index-1][0],
                path[single_index-1][1],
                new_direction,
                "delay by turn left"])
                single_path.append(path[single_index])
            #need turn right first
            elif (curr_direction - pre_direction == 1 or (curr_direction == 0 and pre_direction == 3)):
                new_direction = (pre_direction + 1)%4
                single_path.append([path[single_index-1][0],
                path[single_index-1][1],
                new_direction,
                "delay by turn right"])
                single_path.append(path[single_index])
            #need turn 180
            elif (curr_direction - pre_direction == 2 or curr_direction - pre_direction == -2):
                new_direction = (pre_direction + 1)%4
                single_path.append([path[single_index-1][0],
                path[single_index-1][1],
                new_direction,
                "delay by turn 180 step 1"])

                new_direction = (new_direction + 1)%4
                single_path.append([path[single_index-1][0],
                path[single_index-1][1],
                new_direction,
                "delay by turn 180 step 2"])
                single_path.append(path[single_index])
            else:
                print("error")
            single_index+=1
        new_path.append(single_path)
    return new_path

def simulate(scen_path, scen_order):
    time = 1
    sum_of_cost = len(scen_path)
    current_up_to = [0] * len(scen_path)

    occupy = {}

    max_line = 0
    for path in scen_path:
        if (len(path)>max_line):
            max_line = len(path)
    
    #start the simulation
    step = 1
    while step < max_line:
        for i in range(len(scen_path)):
            if step >= len(scen_path[i]):
                continue
            current_x = scen_path[i][step][0]
            current_y = scen_path[i][step][1]
            current_direction = scen_path[i][step][2]
            previous_x = scen_path[i][step-1][0]
            previous_y = scen_path[i][step-1][1]
            previous_direction = scen_path[i][step-1][2]
            loc = str(current_x)+","+str(current_y)
            previous_loc = str(previous_x)+","+str(previous_y)
            #wait or turning option
            if (previous_x == current_x and previous_y == current_y):
                continue
            else:
                #forward option
                if current_direction == previous_direction:
                    if (loc not in scen_order.keys()):
                        continue
                    up_to = scen_order[loc]
                    if (up_to[0] == i):
                        if scen_order[previous_loc][0] == i:
                            scen_order[previous_loc] = scen_order[previous_loc][1:]
                            continue
                        else:
                            print("error in lookup previous",i,scen_order[previous_loc],previous_loc)
                    else:
                        #has to wait to have order
                        scen_path[i].insert(step,[previous_x,previous_y,previous_direction,"wait for order"])
                        if max_line < len(scen_path[i]):
                            max_line = len(scen_path[i])
                else:
                    print("error")
                    #if no occupier, just continue the current step
                    # if loc not in occupy.keys() or occupy[loc] == i:
                    #     continue
                    # elif occupy[loc] != i:
                    #if order is right
        step +=1
    #return new_path  

def validate_path(scen_path):
    max_line = 0
    for path in scen_path:
        if (len(path)>max_line):
            max_line = len(path)
    for step in range(max_line):
        temp_dict = {}
        for i in range(len(scen_path)):
            if (step >= len(scen_path[i])):
                continue
            curr_location = str(scen_path[i][step][0])+","+ str(scen_path[i][step][1])
            if curr_location in temp_dict.keys():
                print("error, agent",i,"and agent",temp_dict[curr_location],"collict at location",curr_location)
            else:
                temp_dict[curr_location] = i 
    


num_of_agent = 10
sum_of_cost_previous = [0] * 25
sum_of_cost_mcp = [0] * 25

paths = readpath(num_of_agent)

for key in paths.keys():
    for i in range(len(paths[key])):
        #print(key)
        sum_of_cost_previous[key-1] += len(paths[key][i])-1

read_init_direction(num_of_agent,paths)
for key in paths.keys():
    for i in range(len(paths[key])):
        add_directions(paths[key][i])

scen_orders = {}

for key in paths.keys():
    scen_orders[key] = preprocess_orders(paths[key])
delayed_path = {}

for key in paths.keys():
    delayed_path[key] = build_delay(paths[key])

# mcp_paths = {}
for key in paths.keys():
    simulate(delayed_path[key],scen_orders[key])
# simulate(delayed_path[25],scen_orders[25])

#simulate(delayed_path[25],scen_orders[25])
#simulate(delayed_path[10],scen_orders[10])

for key in paths.keys():
    validate_path(delayed_path[key])

for key in delayed_path.keys():
    for i in range(len(delayed_path[key])):
        sum_of_cost_mcp[key-1] += len(delayed_path[key][i])-1
print(sum_of_cost_previous)
print(sum_of_cost_mcp)

# print(len(delayed_path[1][0]))


#print(delayed_path[1])

#print(paths[1])
