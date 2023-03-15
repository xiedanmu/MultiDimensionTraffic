import os
import argparse
from Abnormal_Traffic_HWP import ModelOutput, ModelStart
from Library.package_platformAPI.ExternalInterfaceImpl import ExternalInterfaceImpl

parser = argparse.ArgumentParser()
parser.add_argument("scenario_name", type=str, help="Name of the scenario (to identify map and folder for track "
                                                        "files)",default='Cross-Dislocation', nargs="?")
args = parser.parse_args()

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
maps_dir = os.path.join(root_dir, "maps")

lanelet_map_ending = ".net.xml"
lanelet_map_file = os.path.join(maps_dir, args.scenario_name + lanelet_map_ending)

timestamp=0
ExternalInterface= ExternalInterfaceImpl
ExternalInterface.initial(lanelet_map_file)
userdata = dict()
userdata["VehicleDensity"] = 20  # 车俩密度
userdata["timestamp"] = timestamp
ModelStart(userdata)

while timestamp<1:
    result_dict = ModelOutput(userdata)
    userdata["timestamp"]+=1