import numpy as np

rng = np.random.Generator(np.random.MT19937(1))
from Library.package_platformAPI.SimPlatformAPI import SimPlatformAPI
from Library.package_dao.Class_Dao import Dao
from Library.package_entity.Class_Trajectory import Trajectory
from Library.package_entity.Class_Vehicle import Vehicle
from Library.package_interface.Algorithm_Interface_Impl import AlgorithmInterfaceImpl
from Library.package_interface.Motivate_Interface_Impl import MotivateInterfaceImpl
from Library.package_entity.Enum_Driving_Mode import DrivingMode
from Library.package_entity.Enum_Types_Of_Road import TypesOfRoad
from Library.package_entity.Enum_Located_Area import LocatedArea
import pandas as pd

from Library.package_dao.Class_UserParameters import UserParameters

from Library.package_interface.Traffic_Interface_Impl import TrafficInterfaceImpl
import logging

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())

MotivateInterface = MotivateInterfaceImpl
TrafficInterface = TrafficInterfaceImpl
AlgorithmInterface = AlgorithmInterfaceImpl
g_listSaveTrajectory = list()
g_dictSaveProfile = dict()


def run_main(userdata, MultiThreadPool):
    global last_time
    # start_time=time.perf_counter()
    # g_dictSaveProfile[userdata["time"]]=dict()
    # g_dictSaveProfile[userdata["time"]]['car num']=Dao.vehicle_dictionary.__len__()
    delta_T = userdata["time"] - last_time
    last_time = userdata["time"]

    Dao.refresh_dao()
    # g_dictSaveProfile[userdata["time"]]['refresh_dao'] = f'{time.perf_counter() - start_time:.8f}'

    # elapsed_time1=time.perf_counter()
    log.info("main begin")
    log.info("update_vehicle_num_in_world begin")
    # 自定义的参数密度大于0的时候，脚本才自行发车
    if userdata["VehicleDensity"] > 0:
        TrafficInterface.vehicle_generation(generateMode=1, initial_density=userdata["VehicleDensity"],
                                            arrivePossible=2)  # arrivePossible表示每秒钟有几个车进来
    # elapsed_time2=time.perf_counter()
    # g_dictSaveProfile[userdata["time"]]['vehicle_generation']=f'{elapsed_time2 - elapsed_time1:.8f}'

    # TrafficInterface.vehicle_generation_flow(flow=2400,v_mean=userdata["MeanSpeed"])
    log.info("update_trajectory_for_all begin")
    # flag=check_no_last_step_trajectory_calculation()

    # TODO 模拟以下的所有步骤
    update_step()

    flag = check_no_last_step_veh_update()
    # elapsed_time1=time.perf_counter()
    # g_dictSaveProfile[userdata["time"]]['check_no_last_step_veh_update'] = f'{elapsed_time1 - elapsed_time2:.8f}'

    if flag == False:
        # log.warning("check_no_last_step_veh_update is False")
        # print('the job in last time step haven\'t done yet,pass this time step')
        # print("当前线程: ", threading.currentThread().ident, "当前进程：", os.getpid(), " 父进程：", os.getppid(),' update_last_time_step_data over')

        # for vehicle in Dao.vehicle_dictionary.values():
        #     SimPlatformAPI.myChangeSpeed(vehicle.id,0,0)
        return
    set_last_step_veh_update_not_done()
    # elapsed_time2 = time.perf_counter()
    # g_dictSaveProfile[userdata["time"]]['set_last_step_veh_update_not_done'] = f'{elapsed_time2 - elapsed_time1:.8f}'
    # print(f'until set_last_step_veh_update_not_done cost:[{elapsed_time2 - elapsed_time1:.8f}s]')

    update_trajectory_for_all(MultiThreadPool, delta_T)
    # elapsed_time1 = time.perf_counter()
    # g_dictSaveProfile[userdata["time"]]['update_trajectory_for_all'] = f'{elapsed_time1 - elapsed_time2:.8f}'
    # print(f'until update_trajectory_for_all         cost:[{elapsed_time1 - elapsed_time2:.8f}s]')

    log.info("updata_location_for_all begin")

    update_location_data_for_all(userdata, delta_T)
    # elapsed_time2 = time.perf_counter()
    # g_dictSaveProfile[userdata["time"]]['updata_location_data_for_all'] = f'{elapsed_time2 - elapsed_time1:.8f}'
    # print(f'until updata_location_data_for_all      cost:[{elapsed_time2 - elapsed_time1:.8f}s]')

    result = submit_location_data_for_all(userdata)
    # elapsed_time1 = time.perf_counter()
    # g_dictSaveProfile[userdata["time"]]['submit_location_data_for_all'] = f'{elapsed_time1 - elapsed_time2:.8f}'
    # print(f'until submit_location_data_for_all      cost:[{elapsed_time1 - elapsed_time2:.8f}s]')
    # generate_data_in_excel_to_visualization(userdata) # 生成CSV数据使用
    # generate_profile_data_in_excel(userdata)  # 生成CSV数据使用

    # g_dictSaveProfile[userdata["time"]]['total'] = f'{time.perf_counter() - start_time:.8f}'
    return result


def update_step():
    for vehicle in Dao.vehicle_dictionary.values():
        # additional step：主车之类的直接跳过
        if vehicle.id == 0:
            continue
        # step1:先看一下有没有上一步没有计算完的
        if vehicle.is_xy_v_updated_lastT == False:
            log.warning("{} vehicle.is_xy_v_updated_lastT==False".format(vehicle.id))
            continue
