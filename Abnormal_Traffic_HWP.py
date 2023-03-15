import cProfile
import pstats
import time
import platform
from TrafficModelInterface import *
from Library.package_dao.Class_UserParameters import UserParameters
from Library.package_dao.Class_Dao import Dao
from Library.package_platformAPI.SimPlatformAPI import SimPlatformAPI
from Library.package_runservice.RunService import run_main,output_vehicle_data_file,output_profile_data
from Library.package_multithread.Class_MultiThread import MultiThreadPool

import logging
log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())
# logging.basicConfig(level=logging.WARNING)
# logging.disable(logging.ERROR)

#用于性能分析
pr = cProfile.Profile()
pr.enable()

def f8(x):
    ret = "%8.3f" % x
    if ret != '   0.000':
        return ret
    return "%6dµs" % (x * 10000000)

pstats.f8 = f8
#用于性能分析

def ModelStart(userdata):
    log.info('python version of Pano is :{} '.format(platform.python_version()))
    log.info("start here")
    time.sleep(2)
    SimPlatformAPI.setTargetAPI('PanoAPI')

    Dao.init_from_begin(userdata)
    Global_in = userdata['parameters']

    userdata["Seed"] = float(Global_in["Seed"])
    userdata["VehicleDensity"] = float(Global_in["VehicleDensity"])
    userdata["MeanSpeed"] = float(Global_in["MeanSpeed"])

    userdata["MeanFrictionCoefficient"] = float(Global_in["MeanFrictionCoefficient"])
    userdata["MeanCurvature"] = float(Global_in["MeanCurvature"])
    userdata["MeanSlope"] = float(Global_in["MeanSlope"])
    userdata["Visibility"] = float(Global_in["Visibility"])

    userdata["CautiousRatio"] = float(Global_in["CautiousRatio"])
    userdata["CommonRatio"] = float(Global_in["CommonRatio"])
    userdata["RadicalRatio"] = float(Global_in["RadicalRatio"])

    userdata["CarRatio"] = float(Global_in["CarRatio"])
    userdata["BusRatio"] = float(Global_in["BusRatio"])
    userdata["OtherRatio"] = float(Global_in["OtherRatio"])

    userdata["EmergencyRatio"] = float(Global_in["EmergencyRatio"])
    userdata["CommonRatio"] = float(Global_in["CommonRatio"])
    userdata["CasualRatio"] = float(Global_in["CasualRatio"])
    # if not hasattr(sys, 'argv'):
    #     sys.argv = ['']
    # new_car=SimPlatformAPI.myAddVehicle(398,185,10)

def ModelOutput(userdata):
    #time_start=time.perf_counter()
    print('This time is:[', userdata['time'], ']')
    print('Number of vehicles the script takes over is:',Dao.vehicle_dictionary.__len__())
    log.info("@....output here,time is:{}....@".format(userdata["time"]))
    if not Dao.init_finished:
        log.warning("Dao init not finished")
        return

    Dao.refresh_dao() #似乎是多余的

    # # 单车测试
    # curTime = userdata["time"]
    # if curTime<1000:
    #     for vehicle in Dao.vehicle_dictionary.values():
    #         SimPlatformAPI.myMoveTo(vehicle.id, 398, 200, 180)
    #     return

    # print("---test---")
    # test()
    log.info("---run main---")

    run_main(userdata,MultiThreadPool)
    log.info("---run main end---")
    #print(f'cost:{time.perf_counter() - time_start:.8f}s')

def ModelTerminate(userdata):
    # export_lane_shape_information(userdata)
    #output_vehicle_data_file()
    pr.disable()
    stats = pstats.Stats(pr)
    stats.sort_stats('cumulative')  # 'cumulative','time'
    stats.print_stats()
    stats.dump_stats(filename='C:\\Users\\47551\\Desktop\\temp\\profile\\model_profiling.prof')
    # output_profile_data()
    pass
