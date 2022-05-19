#run experiences with different instances
#the agents num is fixed in the program (30)

import subprocess
import os

for i in range(25):
    cmd = "./cbs -m ./random_map/random-32-32-20.map -a ./random_map/random-32-32-20-random-" + str(i+1) + ".scen -o ./mcp/10agentstest.csv --outputPaths=./mcp/10agents/10agentspaths" + str(i+1)+ ".txt -k 10 -t 60"
    p = subprocess.Popen(cmd,shell=True)
    p.wait()
    print("the",i,"runs")

for i in range(25):
    cmd = "./cbs -m ./random_map/random-32-32-20.map -a ./random_map/random-32-32-20-random-" + str(i+1) + ".scen -o ./mcp/15agentstest.csv --outputPaths=./mcp/15agents/15agentspaths" + str(i+1)+ ".txt -k 15 -t 60"
    p = subprocess.Popen(cmd,shell=True)
    p.wait()
    print("the",i,"runs")

for i in range(25):
    cmd = "./cbs -m ./random_map/random-32-32-20.map -a ./random_map/random-32-32-20-random-" + str(i+1) + ".scen -o ./mcp/20agentstest.csv --outputPaths=./mcp/20agents/20agentspaths" + str(i+1)+ ".txt -k 20 -t 60"
    p = subprocess.Popen(cmd,shell=True)
    p.wait()
    print("the",i,"runs")

for i in range(25):
    cmd = "./cbs -m ./random_map/random-32-32-20.map -a ./random_map/random-32-32-20-random-" + str(i+1) + ".scen -o ./mcp/25agentstest.csv --outputPaths=./mcp/25agents/25agentspaths" + str(i+1)+ ".txt -k 25 -t 60"
    p = subprocess.Popen(cmd,shell=True)
    p.wait()
    print("the",i,"runs")

for i in range(25):
    cmd = "./cbs -m ./random_map/random-32-32-20.map -a ./random_map/random-32-32-20-random-" + str(i+1) + ".scen -o ./mcp/30agentstest.csv --outputPaths=./mcp/30agetns/30agentspaths" + str(i+1)+ ".txt -k 30 -t 60"
    p = subprocess.Popen(cmd,shell=True)
    p.wait()
    print("the",i,"runs")