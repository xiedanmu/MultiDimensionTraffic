import cProfile
import pstats
import time
import platform
#from TrafficModelInterface import *
from Library.package_dao.Class_UserParameters import UserParameters
from Library.package_dao.Class_Dao import Dao
# from Library.package_platformAPI.SimPlatformAPI import SimPlatformAPI
from Library import PanoTrafficApi_Python as PanoTrafficAPI
from Library.package_runservice.RunService import run_main_new,output_vehicle_data_file,output_profile_data
# from Library.package_multithread.Class_MultiThread import MultiThreadPool
from Library.package_multithread.Class_MultiThread import SingleThreadCalculation
import  Library.package_platformAPI.PythonTrafficModel_Template as python_interface_hf
import logging
log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())


# logging.basicConfig(level=logging.INFO)
logging.basicConfig(level=logging.WARNING)
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

#全局参数
duration=1

def modelInit(userdata):
    log.info('python version of Pano is :{} '.format(platform.python_version()))
    log.info("start here")
    PanoTrafficAPI.PanoTrafficInterface.PanoTrafficApi_Init("Traffic", "Traffic-A")
    # SimPlatformAPI.setTargetAPI('PanoNewAPI')
    # time.sleep(10)
    # for i in range(0,2000):
    #     log.info("stop here")
    Dao.init_from_begin(userdata)
    # Global_in = userdata['parameters']
    #
    # userdata["Seed"] = float(Global_in["Seed"])
    userdata["VehicleDensity"] = 0
    # userdata["MeanSpeed"] = float(Global_in["MeanSpeed"])
    #
    # userdata["MeanFrictionCoefficient"] = float(Global_in["MeanFrictionCoefficient"])
    # userdata["MeanCurvature"] = float(Global_in["MeanCurvature"])
    # userdata["MeanSlope"] = float(Global_in["MeanSlope"])
    # userdata["Visibility"] = float(Global_in["Visibility"])
    #
    # userdata["CautiousRatio"] = float(Global_in["CautiousRatio"])
    # userdata["CommonRatio"] = float(Global_in["CommonRatio"])
    # userdata["RadicalRatio"] = float(Global_in["RadicalRatio"])
    #
    # userdata["CarRatio"] = float(Global_in["CarRatio"])
    # userdata["BusRatio"] = float(Global_in["BusRatio"])
    # userdata["OtherRatio"] = float(Global_in["OtherRatio"])
    #
    # userdata["EmergencyRatio"] = float(Global_in["EmergencyRatio"])
    # userdata["CommonRatio"] = float(Global_in["CommonRatio"])
    # userdata["CasualRatio"] = float(Global_in["CasualRatio"])
    userdata['duration'] = 1
    #python_interface_hf.modelInit(userdata)
    # if not hasattr(sys, 'argv'):
    #     sys.argv = ['']
    # new_car=PanoTrafficAPI.PanoTrafficApi_AddVehicle(398,185,10)


# 针对每一个交通仿真物的模型调用
# targetObjId 被调用的object ID
# objectType  被调用的仿真物类型  0车辆 1行人
# curTime     当前仿真时间，单位s
# simStep     仿真步长，单位s
def modelInstanceUpdate(userdata,targetObjId, objectType, curTime, simStep):
    #time_start=time.perf_counter()
    # log.info('This time is:[', curTime, '],targetObjId is:[', targetObjId, ']')
    # log.info('Number of vehicles the script takes over is:',Dao.vehicle_dictionary.__len__())
    log.info("@....output here,time is:{}....@vehicle id is：{}".format(curTime,targetObjId))


    if not Dao.init_finished:
        log.warning("Dao init not finished")
        log.info("Dao init not finished")
        return

    if targetObjId not in Dao.vehicle_dictionary.keys():
        Dao.insert_vehicle_to_dict(targetObjId)
    # # 单车测试
    # curTime = userdata["time"]
    # if curTime<1000:
    #     for vehicle in Dao.vehicle_dictionary.values():
    #         PanoTrafficAPI.PanoTrafficApi_MoveTo(vehicle.id, 398, 200, 180)
    #     return

    # log.info("---test---")
    # test()
    log.info("---run main---")

    #run_main_new(userdata,SingleThreadCalculation,targetObjId)
    run_main_new(userdata, targetObjId)
    log.info("---run main end---")
    #log.info(f'cost:{time.perf_counter() - time_start:.8f}s')

# 针对全局类的算法，每一个仿真步长调用一次
# 比如：步长内统计车辆数目，动态在某一车道添加车辆
# curTime     当前仿真时间，单位s
# simStep     仿真步长，单位s
def modelGlobalUpdate(userdata, curTime, simStep):
    log.info('Number of vehicles the script takes over is:{}'.format(Dao.vehicle_dictionary.__len__()))
    Dao.refresh_dao()
    return

def modelTerminate(userdata):
    # export_lane_shape_information(userdata)
    #output_vehicle_data_file()
    pr.disable()
    stats = pstats.Stats(pr)
    stats.sort_stats('cumulative')  # 'cumulative','time'
    stats.print_stats()
    stats.dump_stats(filename='C:\\Users\\47551\\Desktop\\temp\\profile\\model_profiling.prof')
    # output_profile_data()
    pass
