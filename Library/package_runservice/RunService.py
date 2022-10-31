import multiprocessing
# import random

import numpy as np
rng = np.random.Generator(np.random.MT19937(1))
from Library.package_platformAPI import PanoSimTrafficAPI2
from Library.package_dao.Class_Dao import Dao
from Library.package_entity.Class_Trajectory import Trajectory
from Library.package_entity.Class_Vehicle import Vehicle
from Library.package_interface.Algorithm_Interface_Impl import AlgorithmInterfaceImpl
from Library.package_interface.Motivate_Interface_Impl import MotivateInterfaceImpl
from Library.package_entity.Enum_Driving_Mode import DrivingMode
from Library.package_entity.Enum_Types_Of_Road import TypesOfRoad
from Library.package_entity.Enum_Located_Area import LocatedArea
import pandas as pd

from Library.package_interface.Traffic_Interface_Impl import TrafficInterfaceImpl
import logging
log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())


MotivateInterface = MotivateInterfaceImpl
TrafficInterface = TrafficInterfaceImpl
AlgorithmInterface = AlgorithmInterfaceImpl
g_listSaveTrajectory = []


# if __name__ == '__main__':
#     manager = multiprocessing.Manager()


def test():
    # # 测试GetInternalFoeLanes接口
    # for lane in Dao.lane_dictionary.values():
    #     if lane.type == TypesOfRoad.INTERNAL_LANE:  # Internal lane
    #         foe_lanes_and_types = PanoSimTrafficAPI2.myGetInternalFoeLanes(lane.id)
    #         if foe_lanes_and_types:
    #             print("the value is not None",lane.id)
    #             for foe_lane_and_type in foe_lanes_and_types:
    #                 print("current lane is", foe_lane_and_type[0])
    #                 print('the type is ',foe_lane_and_type[1].value)
    #         else:
    #             print("the value is  None", lane.id)
    pass

# 多进程运行函数，目前不能使用
def run_main_multiprocess(userdata):
    log.info("main begin")
    with multiprocessing.Manager() as manager:
        for vehicle in Dao.vehicle_dictionary.values():
            trajectory = Dao.get_trajectory_by_id(vehicle.id)
            shared_memory = manager.list([vehicle, trajectory])  # index 0 is vehicle，index 1 is trajectory
            new_process = multiprocessing.Process(target=update_location(), args=(shared_memory[0], shared_memory[1]))
            new_process.start()
            '''
            lane_id = PanoSimTrafficAPI2.myGetLaneID(vehicle.id)           
            print(
                '------vehicle:' + str(vehicle.id) + ',current mode:' + str(vehicle.driving_mode) + ',current lane:' + str(
                    lane_id) + '--------')
            '''


def update_trajectory_for_all(MultiThreadPool):
    for vehicle in Dao.vehicle_dictionary.values():
        if vehicle.id == 0:
            continue


        #TODO 后面把这一段改成try catch形式
        cur_lane_id = PanoSimTrafficAPI2.myGetLaneID(vehicle.id)
        cur_lane_max_s = TrafficInterface.myGetLaneLength(cur_lane_id)
        if cur_lane_id is None or cur_lane_max_s<-10000:
            vehicle.is_xy_v_updated_lastT = True
            continue

        is_dead_end = PanoSimTrafficAPI2.myIsDeadEnd(cur_lane_id)
        s = PanoSimTrafficAPI2.myGetDistanceFromLaneStart(vehicle.id)
        if cur_lane_id is not None and s > cur_lane_max_s:
            s = cur_lane_max_s
        elif s <= 0:
            s = 0

        if cur_lane_id is not None and is_dead_end and (cur_lane_max_s - s) < 10:
            vehicle.is_xy_v_updated_lastT = True
            continue


        trajectory = Dao.get_trajectory_by_id(vehicle.id)

        # 判断冲突区域  """当前道路的种类会影响到选择加速度更新方式，默认为normal方式"""
        try: #先取trajectory中记录的点
            current_lane_id=trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0][2]
        except Exception as e:
            log.info('some Exception caught in GetVehicleLane {}'.format(e))
            current_lane_id = PanoSimTrafficAPI2.myGetVehicleLane(vehicle.id)

        try:
            current_lane = Dao.lane_dictionary[current_lane_id]
        except KeyError:
            log.error("KeyError: 没有在Dao中找到这条路 {}".format(current_lane_id)) # 例如这种路(':gneJ0_0_2',)就不在Dao中
        else:
            if vehicle.current_lane!=current_lane:#vehicle.current_lane是上一个仿真步的Lane
                vehicle.lane_is_changed=True #设置Route需要用到的

            # # 特殊逻辑，应对3车道变两车道
            # if not PanoSimTrafficAPI2.myIsDeadEnd(current_lane_id) and not current_lane.next_lane and vehicle.driving_mode==DrivingMode.FOLLOW_DRIVING:
            #     trajectory.x_y_laneid_s_l_cums_cuml_yaw_set=[]
            #     continue

            # 区域设置
            if current_lane.type == TypesOfRoad.INTERNAL_LANE: # 现在判断lane的类型除了lane类中的type字段外，还可以用类本身是不是internal_lane
                vehicle.located_area = LocatedArea.INTERNAL
                vehicle.current_lane = current_lane
            elif not PanoSimTrafficAPI2.myIsDeadEnd(current_lane_id) and not current_lane.next_lane: #对应高速匝道向主干道会车(干道上的辅助汇入车道)
                vehicle.located_area = LocatedArea.NORMAL
                vehicle.current_lane = current_lane
                if PanoSimTrafficAPI2.myGetDistanceFromLaneStart(vehicle.id)<5+2*vehicle.random_coefficients or PanoSimTrafficAPI2.myGetDistanceToLaneEnd(vehicle.id)<vehicle.current_speed/3.6*trajectory.change_lane_needed_time: #修复车道连接处 车道判定不一致的问题
                    #rng.integers(10, 15)是为了不要刚出匝道就汇入主干道
                    pass
                elif not vehicle.driving_mode==DrivingMode.LEFT_CHANGING_LANE or not vehicle.driving_mode == DrivingMode.RIGHT_CHANGING_LANE:
                    left_lane_id = PanoSimTrafficAPI2.myGetLeftLaneIDBasedLane(current_lane_id)
                    if left_lane_id != '' and Dao.lane_dictionary[left_lane_id].next_lane:
                        vehicle.located_area=LocatedArea.FORCE_CHANGE_TO_LEFT
                        trajectory.delete_trajectory_data() # 需要强制换道了
                    else:
                        right_lane_id = PanoSimTrafficAPI2.myGetRightLaneIDBasedLane(current_lane_id)
                        if right_lane_id != '' and Dao.lane_dictionary[right_lane_id].next_lane:
                            vehicle.located_area = LocatedArea.FORCE_CHANGE_TO_RIGHT
                            trajectory.delete_trajectory_data() # 需要强制换道了 TODO 这样直接删除 会导致拼接trajectory没有数据可用
            elif PanoSimTrafficAPI2.myGetDistanceToLaneEnd(vehicle.id) < 60:
                vehicle.located_area = LocatedArea.ADJACENT_JUNCTION_AREA
                vehicle.current_lane = current_lane
                # if vehicle.driving_mode==DrivingMode.LEFT_CHANGING_LANE or vehicle.driving_mode==DrivingMode.RIGHT_CHANGING_LANE:
                #    MotivateInterface.generate_trajectory_point_set(vehicle)
            else:
                vehicle.located_area = LocatedArea.NORMAL
                vehicle.current_lane = current_lane
        # 判断冲突区域结束  """当前道路的种类会影响到选择加速度更新方式，默认为normal方式"""

        # 计算轨迹类
        length_of_trajectory = len(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set)
        if length_of_trajectory < 30:
            trajectory.calculate_done = False
            vehicle.is_calculate_trajectory = True  # 考虑弃用
            log.info("Trajectory length of car {} is #{}#".format(vehicle.id,len(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set)))
            MotivateInterface.generate_trajectory_point_set(vehicle, trajectory, MultiThreadPool)
        else:
            vehicle.is_calculate_trajectory = False  # 考虑弃用


def updata_location_data_for_all():
    queue=[]
    for vehicle in Dao.vehicle_dictionary.values():
        if vehicle.is_xy_v_updated_lastT is True:
            continue
        update_location_data_each(vehicle,queue)
        # while len(queue) != 0:
        #     vehicle = queue.pop(0)
        #     update_location_each(vehicle, queue)

    while len(queue)!=0:
        vehicle=queue.pop(0)
        update_location_data_each(vehicle,queue)

def update_location_data_each(vehicle, queue):

    if vehicle.id == 0:

        return

    """"检查轨迹计算是否完毕"""
    trajectory = Dao.get_trajectory_by_id(vehicle.id)
    if not trajectory.calculate_done:
        # print('calculate trajectory is not done '+str(vehicle.id))
        # log.warning("{} calculate trajectory is not done {}".format(vehicle.id,trajectory.calculate_done))
        # trajectory.suspend_time=trajectory.suspend_time+1
        # if trajectory.suspend_time>50:
        #     log.critical("{} trajectory suspend time over than 50 {}, skip this time".format(vehicle.id,trajectory.calculate_done))
        #     # ·········time.sleep(1)
        #     trajectory.suspend_time = 0
        #     return
        queue.append(vehicle)
        return

    # trajectory.suspend_time = 0

    # cur_lane_id = PanoSimTrafficAPI2.myGetLaneID(vehicle.id)
    # cur_lane_max_s = TrafficInterface.myGetLaneLength(cur_lane_id)
    # is_dead_end = PanoSimTrafficAPI2.myIsDeadEnd(cur_lane_id)
    # s = PanoSimTrafficAPI2.myGetDistanceFromLaneStart(vehicle.id)
    # if s > cur_lane_max_s:
    #     s = cur_lane_max_s
    # elif s <= 0:
    #     s = 0

    # if is_dead_end and (cur_lane_max_s - s) < 10:
    #     vehicle.is_xy_v_updated_lastT = True
    #     return

    #更新车辆所在区域
    # current_x=PanoSimTrafficAPI2.myGetVehicleX(vehicle.id)
    # current_y=PanoSimTrafficAPI2.myGetVehicleY(vehicle.id)
    # current_lane_id = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0][2]
    # current_lane = Dao.lane_dictionary[current_lane_id]
    #TrafficInterface.update_the_vehicle_area(current_lane, vehicle) #TODO 这个地方和轨迹规划的更新车辆位置重复了，可以删去

    # # 如果没有点集就返回
    # if len(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set)==0:
    #     return

    # 每个仿真步都会重新计算速度和加速度，但不会计算轨迹
    # 用新的速度得到的s去轨迹中找到离之最近的点，用这个点来更新当前位置
    (x, y, laneid, s, l, cums, cuml, yaw) = MotivateInterface.pop_point_in_trajectory(vehicle, trajectory)

    if vehicle.going_to_update_Route == None:
        ValidDirections=PanoSimTrafficAPI2.myGetValidDirections(vehicle.current_lane.id)
        if len(ValidDirections)==0:
            vehicle.going_to_update_Route = PanoSimTrafficAPI2.myTranslateDirectionToRoute(PanoSimTrafficAPI2.myTranslateIntTo_next_junction_direction(0))
        else:
            vehicle.going_to_update_Route = PanoSimTrafficAPI2.myTranslateDirectionToRoute(rng.choice(ValidDirections))
        vehicle.lane_is_changed = False

    elif vehicle.lane_is_changed:
        # direction=TrafficInterface.get_direction_from_lane_to_lane(vehicle.current_lane.id,laneid)
        # vehicle.going_to_update_Route = PanoSimTrafficAPI2.myTranslateDirectionToRoute(direction)
        if vehicle.current_lane.type==TypesOfRoad.INTERNAL_LANE:
            vehicle.lane_is_changed = False

        else:
            ValidDirections = PanoSimTrafficAPI2.myGetValidDirections(vehicle.current_lane.id)
            if len(ValidDirections) == 0:
                vehicle.going_to_update_Route = PanoSimTrafficAPI2.myTranslateDirectionToRoute(
                    PanoSimTrafficAPI2.myTranslateIntTo_next_junction_direction(0))
            else:
                vehicle.going_to_update_Route = PanoSimTrafficAPI2.myTranslateDirectionToRoute(
                    rng.choice(ValidDirections))
            vehicle.lane_is_changed = False

    vehicle.going_to_update_x=x
    vehicle.going_to_update_y=y
    vehicle.going_to_update_yaw=yaw
    # print("根据轨迹直接更新vehicle的坐标和角度")
    # if x is not None and y is not None and s is not None and yaw is not None:
    

    # print(" move to loaded")
    
    
    # 把进程间的共享内存里的信息保存回Dao层中 不需要 重复了
    # Dao.trajectory_dictionary[vehicle.id] = trajectory
    # Dao.vehicle_dictionary[vehicle.id] = vehicle

    # """"检查是否完毕"""
    # try:
    #     (x, y, laneid, s, l, cums, cuml, yaw) = MotivateInterface.pop_point_in_trajectory(vehicle, trajectory)
    # except Exception as e:
    #     print('some Exception caught in pop_point_in_trajectory', e)
    #     return
    # else:
    #     # print("根据轨迹直接更新vehicle的坐标和角度")
    #     # if x is not None and y is not None and s is not None and yaw is not None:
    #     MotivateInterface.move_to(vehicle.id, x, y, yaw)
    #
    #     # 把进程间的共享内存里的信息保存回Dao层中
    #     Dao.trajectory_dictionary[vehicle.id] = trajectory
    #     Dao.vehicle_dictionary[vehicle.id] = vehicle


def submit_location_data_for_all(userdata):
    len_of_start_in_submit_location_data = len(Dao.vehicle_dictionary)
    count=0

    for vehicle in Dao.vehicle_dictionary.values():
        if vehicle.is_xy_v_updated_lastT is True:
            count+=1
            continue
        vehicle.set_acceleration(vehicle.going_to_update_acc)
        vehicle.set_speed(vehicle.going_to_update_speed)
        PanoSimTrafficAPI2.myChangeSpeed(vehicle.id, vehicle.current_speed, 0)  # todo 这里的第三个参数设为0和1有什么直观区别
        PanoSimTrafficAPI2.myChangeRoute(vehicle.id, vehicle.going_to_update_Route)
        MotivateInterface.move_to(vehicle.id, vehicle.going_to_update_x, vehicle.going_to_update_y, vehicle.going_to_update_yaw)

        vehicle.current_X = vehicle.going_to_update_x
        vehicle.current_Y = vehicle.going_to_update_y
        vehicle.current_Yaw = vehicle.going_to_update_yaw
        vehicle.current_Route=vehicle.going_to_update_Route
        vehicle.is_xy_v_updated_lastT = True

        count += 1

    # if count==len_of_start_in_submit_location_data:
    #     print('All car have been processed  in this time step')

def run_main(userdata, MultiThreadPool):
    log.info("main begin")
    log.info("update_vehicle_num_in_world begin")
    TrafficInterface.vehicle_generation(generateMode=1, initial_density=userdata["VehicleDensity"], arrivePossible=2) #arrivePossible表示每秒钟有几个车进来

    log.info("update_trajectory_for_all begin")
    # flag=check_no_last_step_trajectory_calculation()

    flag=check_no_last_step_veh_update()
    if flag==False:
        # log.warning("check_no_last_step_veh_update is False")
        # print('the job in last time step haven\'t done yet,pass this time step')
        # print("当前线程: ", threading.currentThread().ident, "当前进程：", os.getpid(), " 父进程：", os.getppid(),' update_last_time_step_data over')

        # for vehicle in Dao.vehicle_dictionary.values():
        #     PanoSimTrafficAPI2.myChangeSpeed(vehicle.id,0,0)
        return
    set_last_step_veh_update_not_done()

    update_trajectory_for_all(MultiThreadPool)
    log.info("updata_location_for_all begin")
    updata_location_data_for_all()
    submit_location_data_for_all(userdata)
    #generate_data_in_excel(userdata) # 生成CSV数据使用

def generate_data_in_excel(userdata):
    log.info("generate data in excel")

    for vehicle in Dao.vehicle_dictionary.values():
        log.info("vehicle.id")
        if vehicle.id == 0:
            continue
        # 导出EXCEL必要
        curTime = userdata["time"]
        driving_mode = vehicle.driving_mode
        is_calculate = vehicle.is_calculate_trajectory
        curLane = PanoSimTrafficAPI2.myGetVehicleLane(vehicle.id)
        curRoute= PanoSimTrafficAPI2.myGetRoute(vehicle.id)
        curYaw = PanoSimTrafficAPI2.myGetVehicleYaw(vehicle.id)
        curS = PanoSimTrafficAPI2.myGetDistanceFromLaneStart(vehicle.id)
        curX = PanoSimTrafficAPI2.myGetVehicleX(vehicle.id)
        curY = PanoSimTrafficAPI2.myGetVehicleY(vehicle.id)
        leader_car = PanoSimTrafficAPI2.myGetLeaderVehicle(vehicle.id)
        left_leader_car = PanoSimTrafficAPI2.myGetLeftLeaderVehicle(vehicle.id)
        right_leader_car = PanoSimTrafficAPI2.myGetRightLeaderVehicle(vehicle.id)
        curL = PanoSimTrafficAPI2.myGetVehicleLateralOffset(vehicle.id)
        curAcc = PanoSimTrafficAPI2.myGetVehicleAccel(vehicle.id)
        curSpeed = PanoSimTrafficAPI2.myGetVehicleSpeed(vehicle.id)
        g_listSaveTrajectory.append([curTime,
                                     vehicle.id,
                                     driving_mode,
                                     is_calculate,
                                     curLane,
                                     curRoute,
                                     curX,
                                     curY,
                                     curYaw,
                                     leader_car,
                                     left_leader_car,
                                     right_leader_car,
                                     curS,
                                     curL,
                                     curSpeed,
                                     curAcc])


def check_no_last_step_veh_update():
    """
    检查所有车在上次仿真步（不一定是前一个10ms）是否更新过，若存在没更新过的车，则当前仿真步什么事情都不做，把时间留出来给没更新过的车更新
    """
    for vehicle in Dao.vehicle_dictionary.values():
        if vehicle.id == 0:
            continue
        if vehicle.is_xy_v_updated_lastT==False:
            log.warning("{} vehicle.is_xy_v_updated_lastT==False".format(vehicle.id))
            return False
    return True

def set_last_step_veh_update_not_done():
    """
    把所有车在上次仿真步（不一定是前一个10ms）是否更新过的标志位设为False
    """
    for vehicle in Dao.vehicle_dictionary.values():
        if vehicle.id == 0:
            continue
        vehicle.is_xy_v_updated_lastT = False

def check_no_last_step_trajectory_calculation():
    for vehicle in Dao.vehicle_dictionary.values():
        if vehicle.id == 0:
            continue
        trajectory = Dao.get_trajectory_by_id(vehicle.id)
        if trajectory.calculate_done == False: #满足此条件说明是有轨迹正在计算中
            return False
    return True

def run_main_old(userdata):
    log.info("main begin")
    for vehicle in Dao.vehicle_dictionary.values():
        if vehicle.id == 0:
            continue
        trajectory = Dao.get_trajectory_by_id(vehicle.id)
        # trajectory.set_forecast_time(random.randint(4,8))
        update_location(vehicle, trajectory)
        '''
        lane_id = PanoSimTrafficAPI2.myGetLaneID(vehicle.id)           
        print(
            '------vehicle:' + str(vehicle.id) + ',current mode:' + str(vehicle.driving_mode) + ',current lane:' + str(
                lane_id) + '--------')
        '''

        # 导出EXCEL必要
        curTime = userdata["time"]
        driving_mode = vehicle.driving_mode
        is_calculate = vehicle.is_calculate_trajectory
        curLane = PanoSimTrafficAPI2.myGetVehicleLane(vehicle.id)
        curYaw = PanoSimTrafficAPI2.myGetVehicleYaw(vehicle.id)
        lead = PanoSimTrafficAPI2.myGetLeaderVehicle(vehicle.id)
        curS = PanoSimTrafficAPI2.myGetDistanceFromLaneStart(vehicle.id)
        curX = PanoSimTrafficAPI2.myGetVehicleX(vehicle.id)
        curY = PanoSimTrafficAPI2.myGetVehicleY(vehicle.id)
        leader_car = PanoSimTrafficAPI2.myGetLeaderVehicle(vehicle.id)
        left_leader_car = PanoSimTrafficAPI2.myGetLeftLeaderVehicle(vehicle.id)
        right_leader_car = PanoSimTrafficAPI2.myGetRightLeaderVehicle(vehicle.id)
        curL = PanoSimTrafficAPI2.myGetVehicleLateralOffset(vehicle.id)
        curAcc = PanoSimTrafficAPI2.myGetVehicleAccel(vehicle.id)
        curSpeed = PanoSimTrafficAPI2.myGetVehicleSpeed(vehicle.id)
        g_listSaveTrajectory.append([curTime,
                                     vehicle.id,
                                     driving_mode,
                                     is_calculate,
                                     curLane,
                                     curX,
                                     curY,
                                     curYaw,
                                     leader_car,
                                     left_leader_car,
                                     right_leader_car,
                                     curS,
                                     curL,
                                     curSpeed,
                                     curAcc])


def output_vehicle_data_file():
    column = ["time", "id", "driving_mode", "is_calculate", "lane", "cur_Route",'x', "y", "yaw", "leader_car", "left_leader_car",
              "right_leader_car", "pos", 'off',
              "speed", "acc"]
    df = pd.DataFrame(g_listSaveTrajectory, columns=column)
    df.to_csv("C:\\Users\\47551\\Desktop\\temp\\fvd_Pano.csv")
    log.info("terminate here")


# 已经弃用
def update_location(vehicle: Vehicle, trajectory: Trajectory):
    """
    if vehicle.driving_mode_last_time_changed:
        MotivateInterface.generate_trajectory_point_set(vehicle)
        vehicle.driving_mode_last_time_changed=False # 已经生成新的路径，消除标志位
    """

    # 判断冲突区域  """当前道路的种类会影响到选择加速度更新方式，默认为normal方式"""
    current_lane_id = PanoSimTrafficAPI2.myGetVehicleLane(vehicle.id)
    try:
        current_lane = Dao.lane_dictionary[current_lane_id]
    except KeyError:
        log.error("KeyError: 没有在Dao中找到这条路".format(current_lane_id)) # 例如这种路(':gneJ0_0_2',)就不在Dao中
    else:
        if current_lane.type == TypesOfRoad.INTERNAL_LANE:
            vehicle.located_area = LocatedArea.INTERNAL
            vehicle.current_lane = current_lane
        elif PanoSimTrafficAPI2.myGetDistanceToLaneEnd(vehicle.id) < 40:
            vehicle.located_area = LocatedArea.ADJACENT_JUNCTION_AREA
            vehicle.current_lane = current_lane
            # if vehicle.driving_mode==DrivingMode.LEFT_CHANGING_LANE or vehicle.driving_mode==DrivingMode.RIGHT_CHANGING_LANE:
            #    MotivateInterface.generate_trajectory_point_set(vehicle)
        else:
            vehicle.located_area = LocatedArea.NORMAL
            vehicle.current_lane = current_lane

    # 计算轨迹类
    if len(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set) < 50:
        vehicle.is_calculate_trajectory = True
        MotivateInterface.generate_trajectory_point_set(vehicle, trajectory)
    else:
        vehicle.is_calculate_trajectory = False
    # 每个仿真步都会重新计算速度和加速度，但不会计算轨迹
    # 用新的速度得到的s去轨迹中找到离之最近的点，用这个点来更新当前位置
    try:
        (x, y, laneid, s, l, cums, cuml, yaw) = MotivateInterface.pop_point_in_trajectory(vehicle, trajectory)
    except Exception as e:
        log.error("some Exception caught in pop_point_in_trajectory {}".format(e))
        return
    else:
        # print("根据轨迹直接更新vehicle的坐标和角度")
        if x is not None and y is not None and s is not None and yaw is not None:
            MotivateInterface.move_to(vehicle.id, x, y, yaw)

        # 把进程间的共享内存里的信息保存回Dao层中
        Dao.trajectory_dictionary[vehicle.id] = trajectory
        Dao.vehicle_dictionary[vehicle.id] = vehicle

# 函数: export_lane_shape_information
# 用途: 导出lane_shape的函数
# [i]: cls
# [o]: void
def export_lane_shape_information(userdata):
    g_listSaveTrajectory = []

    def generate_data_in_excel(userdata):

        log.info("generate data in excel")

        #todo 把最大的tolane找出来
        for lane in Dao.lane_dictionary.values():
            #dicLaneToLanes[lane.id]没有值，它就不是起始位置
            if len(Dao.dicLaneToLanes[lane.id])==0:
                continue
            else:
                current_lane_id = lane.id
                cur_lane_max_s = TrafficInterface.myGetLaneLength(current_lane_id)
                is_dead_end = PanoSimTrafficAPI2.myIsDeadEnd(current_lane_id)
                s = 0

                #  while begin
                lane_to_go = current_lane_id  # calculate_lane變了，lane_to_go才會變
                to_go_lane_max_s = cur_lane_max_s
                forecast_time = 0
                x_y_laneid_s_l_cums_cuml_yaw_set = []
                # 进入计算每个时间间隔路径数据的循环
                if is_dead_end:
                    while s < to_go_lane_max_s:
                        if s > to_go_lane_max_s:
                            break
                        (x, y) = TrafficInterface.getXYFromSL(lane_to_go, s, 0)
                        if x == None or y == None:
                            my_error = ValueError('x or y is none in current dead end lane')
                            raise my_error

                        x_y_laneid_s_l_cums_cuml_yaw_set.append((x, y, lane_to_go, s, 0, s, 0))
                        s = s + 20 * 0.1
                        forecast_time = forecast_time + 0.1  # 10mms仿真步

                    if len(x_y_laneid_s_l_cums_cuml_yaw_set) == 1:
                        log.warning("export lane shape :len of x_y_laneid_s_l_yaw_cums_cuml is 1,error")
                        return
                    else:
                        x_y_laneid_s_l_cums_cuml_yaw_set = \
                            AlgorithmInterface.get_yaw_in_generating_points(x_y_laneid_s_l_cums_cuml_yaw_set)

                else:  # 不是dead end
                    accumulate_s = 0  # 用于迭代计算下一条路用
                    while 1:
                        if s > to_go_lane_max_s:
                            """
                            是时候把下一条lane选进来了
                            """
                            s = s - to_go_lane_max_s
                            accumulate_s = accumulate_s + to_go_lane_max_s
                            if (Dao.lane_dictionary[lane_to_go].is_dead_end == True):
                                break  # dead_end路就不用往下选了
                            log.info("current lane to go is {}".format(lane_to_go))
                            lane_to_go = Dao.lane_dictionary[lane_to_go].next_lane[0]  # todo 后期把这里的0改成随机数
                            to_go_lane_max_s = TrafficInterface.myGetLaneLength(lane_to_go)
                            # to_go_lane_max_s = PanoSimTrafficAPI2.myGetLaneLength(lane_to_go)

                            (x, y) = TrafficInterface.getXYFromSL(lane_to_go, s, 0)
                            if x == None or y == None:
                                my_error = ValueError('x or y is none in current dead end lane')
                                raise my_error
                            x_y_laneid_s_l_cums_cuml_yaw_set.append((x, y, lane_to_go, s, 0, s + accumulate_s, 0))
                            s = s + 20 * 0.01
                            forecast_time = forecast_time + 0.01  # 10ms仿真步

                            continue

                        (x, y) = TrafficInterface.getXYFromSL(lane_to_go, s, 0)
                        if x == None or y == None:
                            my_error = ValueError('x or y is none in current dead end lane')
                            raise my_error
                        x_y_laneid_s_l_cums_cuml_yaw_set.append((x, y, lane_to_go, s, 0, s + accumulate_s, 0))
                        s = s + 20 * 0.1
                        forecast_time = forecast_time + 0.1  # 100ms仿真步

                    x_y_laneid_s_l_cums_cuml_yaw_set = AlgorithmInterface.get_yaw_in_generating_points(
                        x_y_laneid_s_l_cums_cuml_yaw_set)


            # 导出EXCEL必要

            for item in x_y_laneid_s_l_cums_cuml_yaw_set:

                curTime = userdata["time"]
                begin_lane_id = lane.id
                x = item[0]
                y = item[1]
                child_lane_id = item[2]
                s=item[3]
                l=item[4]
                cums=item[5]
                cuml=item[6]
                yaw=item[7]

                g_listSaveTrajectory.append([curTime,begin_lane_id,
                                            x,y,child_lane_id,s,l,cums,cuml,yaw])

    def output_vehicle_data_file():
        column = ["curTime", "begin_lane_id", "x", "y", "child_lane_id", 's', "l", "cums", "cuml","yaw"]
        df = pd.DataFrame(g_listSaveTrajectory, columns=column)
        df.to_csv("C:\\Users\\47551\\Desktop\\temp\\lane_data.csv")
        log.info("terminate here")

    generate_data_in_excel(userdata)
    output_vehicle_data_file()
