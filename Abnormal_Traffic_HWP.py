import time
import platform
from TrafficModelInterface import *

from Library.package_dao.Class_Dao import Dao
from Library.package_runservice.RunService import run_main
from Library.package_multithread.Class_MultiThread import MultiThreadPool

import logging
log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())
# logging.basicConfig(level=logging.WARNING)
# logging.disable(logging.ERROR)

def ModelStart(userdata):
    log.info('python version of Pano is :{} '.format(platform.python_version()))
    log.info("start here")
    time.sleep(2)
    Dao.init_from_begin(userdata)
    # if not hasattr(sys, 'argv'):
    #     sys.argv = ['']
    # new_car=PanoSimTrafficAPI2.myAddVehicle(398,185,10)

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
    #         PanoSimTrafficAPI2.myMoveTo(vehicle.id, 398, 200, 180)
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
    pass
