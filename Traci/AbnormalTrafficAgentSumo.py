import cProfile
import os
import pstats
import sys
import time
import platform
import logging
from Library.package_dao.Class_Dao import Dao
from Library.package_runservice.RunService import run_main, output_vehicle_data_file, export_lane_shape_information
from Library.package_multithread.Class_MultiThread import MultiThreadPool
from Library.package_platformAPI.SimPlatformAPI import SimPlatformAPI


log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())
logging.basicConfig(level=logging.WARNING)

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
    SimPlatformAPI.setTargetAPI('SumoAPI')
    root_dir = os.path.dirname(os.path.abspath(__file__))
    lanelet_map_ending = ".net.xml"
    lanelet_map_file = os.path.join(root_dir, 'hello' + lanelet_map_ending)

    SimPlatformAPI.Behavior.setNetFileInAbPath(lanelet_map_file)

    userdata["VehicleDensity"]=10

    Dao.init_from_begin(userdata)
    # if not hasattr(sys, 'argv'):
    #     sys.argv = ['']
    # new_car=SimPlatformAPI.myAddVehicle(398,185,10)

def ModelOutput(userdata):
    log.info("@....output here,time is:{}....@".format(userdata["time"]))
    if not Dao.init_finished:
        log.warning("Dao init not finished")
        return

    Dao.refresh_dao()

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
    # Dao.update_userdata(userdata)  # 没用


def ModelTerminate(userdata):

    # export_lane_shape_information(userdata)
    # output_vehicle_data_file()
    pr.disable()
    stats = pstats.Stats(pr)
    stats.sort_stats('cumulative')  # 'cumulative','time'
    stats.print_stats()
    stats.dump_stats(filename='C:\\Users\\47551\\Desktop\\temp\\profile\\traci_profiling.prof')
    pass
