import os
import sys
import optparse
import traci
import AbnormalTrafficAgentSumo as atap

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

if __name__ == "__main__":
    options = get_options()   
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    traci.start([sumoBinary, "-c", "hello.sumocfg"])
    userdata={}
    userdata["time"] = traci.simulation.getTime()
    atap.ModelStart(userdata)

    for step in range(0,600):
        userdata["time"] = traci.simulation.getTime()
        atap.ModelOutput(userdata)
#        traci.vehicle.setSpeed('a12.5',10)
#        print(traci.vehicle.getIDList())
#        print(traci.edge.getIDList())
#        print(traci.inductionloop.getVehicleData('abcd'))
        traci.simulationStep()
    atap.ModelTerminate(userdata)
    traci.close()
