import random

def changescen(filename,writefile):
    f = open(filename)

    line = f.readline()
    lines = ['rotation 1\n']
    line = f.readline()
    while (line):
        temp = line.strip().replace("\n","").split("\t")
        #print(temp)
        temp.insert(6,str(random.randint(0,3)))
        temp.insert(9,str(random.randint(0,3)))
        lines.append('\t'.join(temp) + '\n')
        #print(temp)
        line = f.readline()

    f.close()

    file = open(writefile,"w")
    for item in lines:
        file.write(item)
    file.close()


#changescen("instances/random_map_temp/random-32-32-20-random-1.scen","instances/random_map/random-32-32-20-random-1.scen")

for i in range(25):
    r_file = "instances/random_map_temp/random-32-32-20-random-"+str(i+1)+".scen"
    w_file = "instances/random_map/random-32-32-20-random-"+str(i+1)+".scen"
    changescen(r_file,w_file)