import math
# import random
# random.seed(1)
import numpy as np

from Library.package_entity.Class_Foe_Vehicle import FoeVehicle
from Library.package_entity.Class_Internal_Lane import InternalLane
from Library.package_entity.Class_Lane import Lane

rng = np.random.Generator(np.random.MT19937(1))

from Library.package_platformAPI.SimPlatformAPI import SimPlatformAPI
from Library.package_entity.Class_Trajectory import Trajectory
from Library.package_entity.Class_Vehicle import Vehicle
from Library.package_entity.Enum_Driving_Mode import DrivingMode
from Library.package_entity.Enum_Located_Area import LocatedArea
from Library.package_entity.Enum_Types_Of_Road import TypesOfRoad
from Library.package_entity.Enum_Direction import Direction
from Library.package_interface.Motivate_Interface import MotivateInterface
from Library.package_interface.Traffic_Interface_Impl import TrafficInterfaceImpl
from Library.package_dao.Class_Dao import Dao
from Library.package_interface.Algorithm_Interface_Impl import AlgorithmInterfaceImpl

# from package_multithread.Class_MultiThread import MultiThread

TrafficInterface = TrafficInterfaceImpl
AlgorithmInterface = AlgorithmInterfaceImpl

import logging

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())


# MultiThreadExecutor = MultiThread


class MotivateInterfaceImpl(MotivateInterface):

    def __init__(self):
        pass

    @classmethod
    def get_and_update_accelerator(cls, vehicle: Vehicle, trajectory: Trajectory):

        # 换道的时候加速度直接不变
        # if vehicle.driving_mode == DrivingMode.RIGHT_CHANGING_LANE or vehicle.driving_mode == DrivingMode.LEFT_CHANGING_LANE:
        #     # print("正在换道")
        #     return 0

        # 在路口里面时的速度更新方式
        try:
            # current_lane_id = SimPlatformAPI.myGetVehicleLane(vehicle.id)

            # 避免某些地图出现重复lane数据的情况下，用Pano提供的getVehicleLane返回的值是另外一条“同样的”lane的数据，不得已罢了，其实修改地图才是上策
            current_lane_id = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0][2]
            current_lane = Dao.lane_dictionary[current_lane_id]

        except KeyError:
            print('wrong in get_and_update_accelerator')
            log.error("some Exception caught in GetVehicleLane or KeyError: 没有在Dao中找到这条路 {}".format(
                'trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0][2]'))
            # 例如这种路(':gneJ0_0_2',':gneJ2_0_2')就不在Dao中
        else:
            # 如果是换道
            if vehicle.driving_mode == DrivingMode.LEFT_CHANGING_LANE or vehicle.driving_mode == DrivingMode.RIGHT_CHANGING_LANE:
                return cls.get_and_update_accelerator_change_lane(vehicle, trajectory)
            # 在路口内了
            if current_lane.type == TypesOfRoad.INTERNAL_LANE:  # 现在判断lane的类型除了lane类中的type字段外，还可以用类本身是不是internal_lane

                return cls.crossroad_game_model_new(vehicle, trajectory, current_lane)

            # 如果接近路口了
            if vehicle.located_area == LocatedArea.ADJACENT_JUNCTION_AREA:

                dis = SimPlatformAPI.myGetDistanceToLaneEnd(vehicle.id)
                is_dead_end = SimPlatformAPI.myIsDeadEnd(current_lane_id)

                if is_dead_end:
                    return cls.get_and_update_accelerator_normal(vehicle, trajectory)

                next_lane_id = TrafficInterface.get_next_lane_by_trajectory(trajectory)
                # try:
                #     # next_lane_id = Dao.lane_dictionary[current_lane_id].next_lane[0] #替换为了下一行
                #
                #     # 这样获取current lane的目的是为了避免Pano提供的GetNextLane返回的lane并不是trajectory中记录了的lane
                #     next_lane_id=TrafficInterface.get_next_lane_by_trajectory(trajectory)
                # except Exception as e:
                #     print("current lane is: [",current_lane_id,'] some Exception caught in get next lane id', e)
                #     print("next lane of current lane are:",Dao.lane_dictionary[current_lane_id].next_lane)
                #     raise e

                # 是红灯或者黄灯
                # 0:red 1:yellow 2:green 3:unknown
                try:  # 用try 是因为有的路口是没有红绿灯的
                    direction = Dao.dicLaneToLaneDir[(current_lane_id, next_lane_id)]
                    light_state = SimPlatformAPI.myGetTrafficLightState(vehicle.id, direction).value
                except:
                    log.info("some Exception caught in GetVehicleLane or KeyError: 没有在Dao中找到这条路 {}".format(
                        current_lane_id))
                else:
                    # print(current_lane_id," to ",next_lane_id," is ",light_state)
                    # 是绿灯
                    if light_state == 0:
                        return cls.get_and_update_accelerator_stop_line(vehicle, dis, trajectory)
                        # return cls.get_and_update_accelerator_normal_internal(vehicle)

                    if SimPlatformAPI.myGetLeaderVehicle(vehicle.id) > -1:
                        return cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)

                if vehicle.foe_vehicle is not None:
                    if vehicle.located_area != LocatedArea.INTERNAL:
                        # vehicle.foe_vehicle = None
                        foe_s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.foe_vehicle.id)

                        my_s = -SimPlatformAPI.myGetDistanceToLaneEnd(vehicle.id)
                        foe_length = SimPlatformAPI.myGetVehicleLength(vehicle.foe_vehicle.id)
                        my_delta_s = vehicle.foe_vehicle.calculate_my_delta_s(my_s, foe_s, foe_length)

                        if my_delta_s is not None:
                            try:
                                foe_vehicle = Dao.vehicle_dictionary[vehicle.foe_vehicle.id]
                            except:
                                vehicle.foe_vehicle = None
                            else:
                                return cls.get_and_update_accelerator_internal_with_foe(vehicle, trajectory,
                                                                                        foe_vehicle,
                                                                                        my_delta_s)
                        else:
                            vehicle.foe_vehicle = None

                # 判断下一条路的冲突关系了（这里一般是internallane）
                next_lane = Dao.lane_dictionary[next_lane_id]

                try:
                    next_lane.foe_lane_type_points_s
                except:
                    log.info("this lane doesn't have foe_lane_type_points_s:{}".format(next_lane_id))
                else:
                    leader_id = SimPlatformAPI.myGetLeaderVehicle(vehicle.id)
                    if leader_id > -1:  # 停止线之前有leader
                        return cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)

                    my_s = -SimPlatformAPI.myGetDistanceToLaneEnd(vehicle.id)
                    temp_foe_vehicle_map_to_my_lane_s = dict()
                    temp_foe_vehicle_related_data = dict()  # alpha,intersection_s_of_my_lane,intersection_s_of_foe_lane,foe_car_s_each,foe_lane_type_points_s_item[3]

                    for foe_lane_type_points_s_item in next_lane.foe_lane_type_points_s:
                        s_of_intersection_in_my_lane = foe_lane_type_points_s_item[3]

                        foe_lane_id = foe_lane_type_points_s_item[0]
                        foe_lane = Dao.lane_dictionary[foe_lane_id]
                        foe_cars = SimPlatformAPI.myGetLaneVehicles(foe_lane_id)  # 获取foe lane 上的所有车

                        if not foe_cars:
                            continue

                        my_s_to_intersection = foe_lane_type_points_s_item[3] - my_s  # 我自己到交点的距离
                        intersection_s_of_my_lane = foe_lane_type_points_s_item[3]
                        alpha = foe_lane_type_points_s_item[4]
                        intersection_s_of_foe_lane = 0
                        foe_s_to_intersection = 0

                        # 找到foe_lane对应的交汇处的s
                        for foe_lane_type_points_s_item_of_foe in foe_lane.foe_lane_type_points_s:
                            if foe_lane_type_points_s_item_of_foe[0] == next_lane.id:
                                intersection_s_of_foe_lane = foe_lane_type_points_s_item_of_foe[3]
                                break

                        for foe_car_each_id in foe_cars:
                            foe_car_s_each = SimPlatformAPI.myGetDistanceFromLaneStart(foe_car_each_id)

                            rest_foe_s = intersection_s_of_foe_lane - foe_car_s_each
                            map_to_my_lane_s = foe_lane_type_points_s_item[3] - rest_foe_s  # 映射到我的路上，foe车的s是多少
                            if map_to_my_lane_s < my_s:  # 超过我的s了，不考虑
                                continue
                            else:
                                temp_foe_vehicle_map_to_my_lane_s[foe_car_each_id] = map_to_my_lane_s
                                temp_foe_vehicle_related_data[foe_car_each_id] = (alpha,
                                                                                  intersection_s_of_my_lane,
                                                                                  intersection_s_of_foe_lane,
                                                                                  foe_car_s_each,
                                                                                  s_of_intersection_in_my_lane,
                                                                                  intersection_s_of_foe_lane)
                    if temp_foe_vehicle_map_to_my_lane_s.__len__() != 0:
                        target_foe_vehicle_id = min(temp_foe_vehicle_map_to_my_lane_s,
                                                    key=lambda x: temp_foe_vehicle_map_to_my_lane_s[x])  # 找到最小值对应的KEY
                        foe_vehicle = Dao.vehicle_dictionary[target_foe_vehicle_id]

                        my_width = SimPlatformAPI.getVehicleWidth(vehicle.vehicle_type)
                        foe_width = SimPlatformAPI.getVehicleWidth(foe_vehicle.vehicle_type)
                        foe_length = SimPlatformAPI.myGetVehicleLength(target_foe_vehicle_id)
                        alpha = temp_foe_vehicle_related_data[target_foe_vehicle_id][0]
                        intersection_s_of_my_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][1]
                        intersection_s_of_foe_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][2]
                        foe_car_s_each = temp_foe_vehicle_related_data[target_foe_vehicle_id][3]
                        s_of_intersection_in_my_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][4]
                        s_of_intersection_in_foe_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][5]

                        foe_s_to_entry_conflict_zone, foe_s_to_leave_conflict_zone, my_stop_s_at_entry_conflict_zone = \
                            FoeVehicle.calculate_foe_entry_and_leave_data(alpha, intersection_s_of_my_lane,
                                                                          intersection_s_of_foe_lane, my_width,
                                                                          foe_width, foe_length)
                        if foe_car_s_each > foe_s_to_leave_conflict_zone:
                            # vehicle.foe_vehicle=None
                            return cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)
                        else:
                            vehicle.foe_vehicle = FoeVehicle(target_foe_vehicle_id,
                                                             foe_s_to_entry_conflict_zone,
                                                             foe_s_to_leave_conflict_zone,
                                                             my_stop_s_at_entry_conflict_zone,
                                                             s_of_intersection_in_my_lane,
                                                             s_of_intersection_in_foe_lane,
                                                             alpha)

                            my_s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
                            my_delta_s = vehicle.foe_vehicle.calculate_my_delta_s(my_s, foe_car_s_each, foe_length)

                            return cls.get_and_update_accelerator_internal_with_foe(vehicle, trajectory, foe_vehicle,
                                                                                    my_delta_s)

                return cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)

            # IDM模型
            else:
                # 一般情况下
                vehicle.foe_vehicle = None
                return cls.get_and_update_accelerator_normal(vehicle, trajectory)
            # return cls.get_and_update_accelerator_normal(vehicle, trajectory)
        # 判断冲突区域结束

    @classmethod
    def crossroad_game_model_old(cls, vehicle: Vehicle, trajectory: Trajectory, current_lane: InternalLane):

        # print("Internal [",vehicle.id, "] is on ", current_lane_id)
        s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
        # 只判断最近的车的写法
        potential_foe_vehicles_inter_s = []  # {冲突车，冲突车交点距离，当前车到交点距离}

        # 只用计算前方三条冲突路需要用到 ，但是只计算三条不够
        i = 3
        calculate_count = 0
        for foe_lane_type_points_s_item in current_lane.foe_lane_type_points_s:
            # 只用计算前方三条冲突路
            # calculate_count=calculate_count+1
            # if calculate_count>=3:
            #     break

            if s < foe_lane_type_points_s_item[3]:

                leader = SimPlatformAPI.myGetLeaderVehicle(vehicle.id)
                if leader > -1:
                    leader_s = SimPlatformAPI.myGetDistanceFromLaneStart(leader)
                    if leader_s < foe_lane_type_points_s_item[3]:
                        continue

                real_foe_car = None  # temp变量
                rest_foe_s = 0  # foe车到交点的距离加上车身长度
                foe_s = 0  # 交点的s值

                foe_lane_id = foe_lane_type_points_s_item[0]
                foe_lane = Dao.lane_dictionary[foe_lane_id]
                foe_cars = SimPlatformAPI.myGetLaneVehicles(foe_lane_id)  # 获取foe lane 上的所有车

                if not foe_cars:
                    continue

                # 找到foe_lane对应的交汇处的s
                for foe_lane_type_points_s_item_of_foe in foe_lane.foe_lane_type_points_s:
                    if foe_lane_type_points_s_item_of_foe[0] == current_lane.id:
                        foe_s = foe_lane_type_points_s_item_of_foe[3]
                        break

                # 在当前冲突lane中比较哪台foe车离给定的S最近，并且没有超出S，每条冲突lane只有一个候选冲突车
                delta = 10000  # 给一个极大值
                foe_car_block_road = False
                for foe_car_each in foe_cars:
                    # foe_car_lane_id = SimPlatformAPI.myGetVehicleLane(foe_car_each)
                    foe_car_s_each = SimPlatformAPI.myGetDistanceFromLaneStart(foe_car_each)
                    total_s = foe_s + SimPlatformAPI.myGetVehicleLength(foe_car_each) + SimPlatformAPI.getVehicleWidth(
                        vehicle.vehicle_type) / 2 + 3  # 车尾离开冲突点总行驶距离，加3是为了留有通行余量
                    # # 车头到foe点的距离
                    # distance_from_head = abs(foe_s - foe_car_s_each)  # TODO 算距离这里还可以简化
                    # 车尾到foe点的距离
                    if foe_car_s_each >= total_s:
                        continue
                    if foe_car_s_each < delta:
                        delta = foe_car_s_each
                        real_foe_car = foe_car_each
                        rest_foe_s = total_s - foe_car_s_each
                        if foe_car_s_each > foe_s - SimPlatformAPI.getVehicleWidth(
                                vehicle.vehicle_type) / 2 - 3:  # -3是为了余量
                            foe_car_block_road = True
                        else:
                            foe_car_block_road = False

                if real_foe_car:
                    foe_vehicle = Dao.vehicle_dictionary[real_foe_car]
                    potential_foe_vehicles_inter_s.append(
                        (foe_vehicle, rest_foe_s, foe_car_block_road,
                         foe_lane_type_points_s_item[3] + SimPlatformAPI.myGetVehicleLength(
                             vehicle.id) + SimPlatformAPI.getVehicleWidth(foe_vehicle.vehicle_type) / 2 - s + 3))
        # print("potential_foe_vehicles_inter_s are:",potential_foe_vehicles_inter_s)

        # 没有搜索到敌对车
        if not potential_foe_vehicles_inter_s:
            # print("在路口内")
            # print("没有搜索到敌对车")
            return cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)
        # 有敌对车
        else:
            # print("在路口内")
            # print("有敌对车")
            all_acc = []
            for item in potential_foe_vehicles_inter_s:
                log.info("{},{},{},{}".format(item[0].id, item[1], item[2], item[3]))
                target_vehicle = item[0]
                foe_rest_s = item[1]
                foe_car_block_road = item[2]
                rest_s = item[3]  # 自己车到离开foe点的距离
                all_acc.append(
                    cls.get_and_update_accelerator_internal_by_distance(vehicle, trajectory, rest_s, target_vehicle,
                                                                        foe_rest_s, foe_car_block_road))

            # print(vehicle.id," accs are:",all_acc)
            return min(all_acc)
            # closest_dis = 10000
            # target_vehicle = None
            # rest_s = 0  # 当前车到交点的距离
            # foe_rest_s = 0  # foe车到交点的距离
            # foe_car_block_road=False
            # for item in potential_foe_vehicles_inter_s:
            #     temp = TrafficInterface.two_car_distance(item[0].id, vehicle.id)
            #     print("temp is",temp)
            #     if temp < closest_dis:
            #         closest_dis = temp
            #         target_vehicle = item[0]
            #         foe_rest_s = item[1]
            #         foe_car_block_road=item[2]
            #         rest_s = item[3]
            # print("target_vehicle is", target_vehicle.id)
            # return cls.get_and_update_accelerator_internal(vehicle, rest_s, target_vehicle, foe_rest_s,foe_car_block_road)

    @classmethod
    def crossroad_game_model_new(cls, vehicle: Vehicle, trajectory: Trajectory, current_lane: InternalLane):

        s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
        temp_foe_vehicle_map_to_my_lane_s = dict()
        temp_foe_vehicle_related_data = dict()  # alpha,intersection_s_of_my_lane,intersection_s_of_foe_lane,foe_car_s_each,foe_lane_type_points_s_item[3]
        find_new_foe_car = False

        # if vehicle.foe_vehicle is not None:
        #     foe_vehicle=Dao.vehicle_dictionary[vehicle.foe_vehicle.id]
        #     if foe_vehicle.located_area!=LocatedArea.INTERNAL:
        #         #vehicle.foe_vehicle = None
        #         foe_s = vehicle.foe_vehicle.s_of_intersection_in_foe_lane+SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.foe_vehicle.id)
        #
        #         my_s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
        #         foe_length = SimPlatformAPI.myGetVehicleLength(vehicle.foe_vehicle.id)
        #         my_delta_s = vehicle.foe_vehicle.calculate_my_delta_s(my_s, foe_s, foe_length)
        #
        #         if my_delta_s is not None:
        #             foe_vehicle = Dao.vehicle_dictionary[vehicle.foe_vehicle.id]
        #             return cls.get_and_update_accelerator_internal_with_foe(vehicle, trajectory, foe_vehicle,
        #                                                                     my_delta_s)
        #         else:
        #             vehicle.foe_vehicle = None
        #     else:
        #         foe_s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.foe_vehicle.id)
        #
        #         my_s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
        #         foe_length = SimPlatformAPI.myGetVehicleLength(vehicle.foe_vehicle.id)
        #         my_delta_s = vehicle.foe_vehicle.calculate_my_delta_s(my_s, foe_s, foe_length)
        #
        #         if my_delta_s is not None:
        #             foe_vehicle = Dao.vehicle_dictionary[vehicle.foe_vehicle.id]
        #             return cls.get_and_update_accelerator_internal_with_foe(vehicle, trajectory, foe_vehicle,
        #                                                                     my_delta_s)
        #         else:
        #             vehicle.foe_vehicle = None

        leader_id = SimPlatformAPI.myGetLeaderVehicle(vehicle.id)
        leader_s = 10000
        if leader_id > -1:
            leader_s = SimPlatformAPI.myGetDistanceFromLaneStart(leader_id)

        for foe_lane_type_points_s_item in current_lane.foe_lane_type_points_s:
            s_of_intersection_in_my_lane = foe_lane_type_points_s_item[3]

            if s < s_of_intersection_in_my_lane and s_of_intersection_in_my_lane < leader_s:  # 这是一条很重要的条件

                foe_lane_id = foe_lane_type_points_s_item[0]
                foe_lane = Dao.lane_dictionary[foe_lane_id]
                foe_cars = SimPlatformAPI.myGetLaneVehicles(foe_lane_id)  # 获取foe lane 上的所有车

                if not foe_cars:
                    continue

                my_s = s
                my_s_to_intersection = foe_lane_type_points_s_item[3] - my_s  # 我自己到交点的距离
                intersection_s_of_my_lane = foe_lane_type_points_s_item[3]
                alpha = foe_lane_type_points_s_item[4]
                intersection_s_of_foe_lane = 0
                foe_s_to_intersection = 0

                # 找到foe_lane对应的交汇处的s
                for foe_lane_type_points_s_item_of_foe in foe_lane.foe_lane_type_points_s:
                    if foe_lane_type_points_s_item_of_foe[0] == current_lane.id:
                        intersection_s_of_foe_lane = foe_lane_type_points_s_item_of_foe[3]
                        break

                for foe_car_each_id in foe_cars:
                    foe_car_s_each = SimPlatformAPI.myGetDistanceFromLaneStart(foe_car_each_id)

                    rest_foe_s = intersection_s_of_foe_lane - foe_car_s_each
                    map_to_my_lane_s = foe_lane_type_points_s_item[3] - rest_foe_s  # 映射到我的路上，foe车的s是多少
                    if map_to_my_lane_s > leader_s or map_to_my_lane_s < my_s:  # 超过我的leader的s了，不考虑；一旦在待转区域，所有车都是我的考虑车
                        continue
                    else:
                        temp_foe_vehicle_map_to_my_lane_s[foe_car_each_id] = map_to_my_lane_s
                        temp_foe_vehicle_related_data[foe_car_each_id] = (alpha,
                                                                          intersection_s_of_my_lane,
                                                                          intersection_s_of_foe_lane,
                                                                          foe_car_s_each,
                                                                          s_of_intersection_in_my_lane,
                                                                          intersection_s_of_foe_lane)

        # if temp_foe_vehicle_map_to_my_lane_s.__len__() !=0 :
        #     target_foe_vehicle_id = min(temp_foe_vehicle_map_to_my_lane_s,
        #                                 key=lambda x: temp_foe_vehicle_map_to_my_lane_s[x])  # 找到最小值对应的KEY
        #     foe_vehicle = Dao.vehicle_dictionary[target_foe_vehicle_id]
        #
        #     my_width = SimPlatformAPI.getVehicleWidth(vehicle.vehicle_type)
        #     foe_width = SimPlatformAPI.getVehicleWidth(foe_vehicle.vehicle_type)
        #     foe_length = SimPlatformAPI.myGetVehicleLength(target_foe_vehicle_id)
        #     alpha = temp_foe_vehicle_related_data[target_foe_vehicle_id][0]
        #     intersection_s_of_my_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][1]
        #     intersection_s_of_foe_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][2]
        #     foe_car_s_each = temp_foe_vehicle_related_data[target_foe_vehicle_id][3]
        #     s_of_intersection_in_my_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][4]
        #     s_of_intersection_in_foe_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][5]
        #
        #     foe_s_to_entry_conflict_zone, foe_s_to_leave_conflict_zone, my_stop_s_at_entry_conflict_zone = \
        #         FoeVehicle.calculate_foe_entry_and_leave_data(alpha, intersection_s_of_my_lane,
        #                                                       intersection_s_of_foe_lane, my_width,
        #                                                       foe_width, foe_length)
        #     if foe_car_s_each > foe_s_to_leave_conflict_zone:
        #         vehicle.foe_vehicle=None
        #         return cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)
        #     else:
        #         vehicle.foe_vehicle = FoeVehicle(target_foe_vehicle_id,
        #                                          foe_s_to_entry_conflict_zone,
        #                                          foe_s_to_leave_conflict_zone,
        #                                          my_stop_s_at_entry_conflict_zone,
        #                                          s_of_intersection_in_my_lane,
        #                                          s_of_intersection_in_foe_lane,
        #                                          alpha)
        #
        #         my_s=SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
        #         my_delta_s = vehicle.foe_vehicle.calculate_my_delta_s(my_s, foe_car_s_each, foe_length)
        #
        #         return cls.get_and_update_accelerator_internal_with_foe(vehicle, trajectory,foe_vehicle, my_delta_s)

        if temp_foe_vehicle_map_to_my_lane_s.__len__() != 0:
            all_my_stop_s_at_entry_conflict_zone = dict()
            for temp_foe_vehicle_id in list(temp_foe_vehicle_related_data.keys()):
                foe_vehicle = Dao.vehicle_dictionary[temp_foe_vehicle_id]

                my_width = SimPlatformAPI.getVehicleWidth(vehicle.vehicle_type)
                foe_width = SimPlatformAPI.getVehicleWidth(foe_vehicle.vehicle_type)
                foe_length = SimPlatformAPI.myGetVehicleLength(temp_foe_vehicle_id)
                alpha = temp_foe_vehicle_related_data[temp_foe_vehicle_id][0]
                intersection_s_of_my_lane = temp_foe_vehicle_related_data[temp_foe_vehicle_id][1]
                intersection_s_of_foe_lane = temp_foe_vehicle_related_data[temp_foe_vehicle_id][2]
                foe_car_s_each = temp_foe_vehicle_related_data[temp_foe_vehicle_id][3]

                foe_s_to_entry_conflict_zone, foe_s_to_leave_conflict_zone, my_stop_s_at_entry_conflict_zone = \
                    FoeVehicle.calculate_foe_entry_and_leave_data(alpha, intersection_s_of_my_lane,
                                                                  intersection_s_of_foe_lane, my_width,
                                                                  foe_width, foe_length)

                if foe_car_s_each > foe_s_to_leave_conflict_zone:
                    continue
                else:
                    all_my_stop_s_at_entry_conflict_zone[temp_foe_vehicle_id] = my_stop_s_at_entry_conflict_zone

            if all_my_stop_s_at_entry_conflict_zone.__len__() == 0:
                vehicle.foe_vehicle = None
                return cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)

            target_foe_vehicle_id = min(all_my_stop_s_at_entry_conflict_zone,
                                        key=lambda x: all_my_stop_s_at_entry_conflict_zone[x])  # 找到最小值对应的KEY
            foe_vehicle = Dao.vehicle_dictionary[target_foe_vehicle_id]

            my_width = SimPlatformAPI.getVehicleWidth(vehicle.vehicle_type)
            foe_width = SimPlatformAPI.getVehicleWidth(foe_vehicle.vehicle_type)
            foe_length = SimPlatformAPI.myGetVehicleLength(target_foe_vehicle_id)
            alpha = temp_foe_vehicle_related_data[target_foe_vehicle_id][0]
            intersection_s_of_my_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][1]
            intersection_s_of_foe_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][2]
            foe_car_s_each = temp_foe_vehicle_related_data[target_foe_vehicle_id][3]
            s_of_intersection_in_my_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][4]
            s_of_intersection_in_foe_lane = temp_foe_vehicle_related_data[target_foe_vehicle_id][5]

            foe_s_to_entry_conflict_zone, foe_s_to_leave_conflict_zone, my_stop_s_at_entry_conflict_zone = \
                FoeVehicle.calculate_foe_entry_and_leave_data(alpha, intersection_s_of_my_lane,
                                                              intersection_s_of_foe_lane, my_width,
                                                              foe_width, foe_length)
            vehicle.foe_vehicle = FoeVehicle(target_foe_vehicle_id,
                                             foe_s_to_entry_conflict_zone,
                                             foe_s_to_leave_conflict_zone,
                                             my_stop_s_at_entry_conflict_zone,
                                             s_of_intersection_in_my_lane,
                                             s_of_intersection_in_foe_lane,
                                             alpha)

            my_s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
            my_delta_s = vehicle.foe_vehicle.calculate_my_delta_s(my_s, foe_car_s_each, foe_length)

            return cls.get_and_update_accelerator_internal_with_foe(vehicle, trajectory, foe_vehicle, my_delta_s)

        else:
            return cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)

    @classmethod
    def crossroad_game_model_ST(cls, vehicle: Vehicle, trajectory: Trajectory, current_lane: InternalLane):
        s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
        # 只判断最近的车的写法
        potential_foe_vehicles_inter_s = []  # {冲突车，冲突车交点距离，当前车到交点距离}

        # 只用计算前方i条冲突路需要用到 ，但是只计算三条不够
        i = 1
        calculate_count = 0
        for foe_lane_type_points_s_item in current_lane.foe_lane_type_points_s:
            # 只用计算前方三条冲突路
            calculate_count = calculate_count + 1
            if calculate_count >= 2:  # 只算一个路
                break

            if s < foe_lane_type_points_s_item[3]:
                real_foe_car = None  # temp变量
                rest_foe_s = 0  # foe车到交点的距离加上车身长度
                foe_s = 0  # 交点的s值

                foe_lane_id = foe_lane_type_points_s_item[0]
                foe_lane = Dao.lane_dictionary[foe_lane_id]
                foe_cars = SimPlatformAPI.myGetLaneVehicles(foe_lane_id)  # 获取foe lane 上的所有车

                if not foe_cars:
                    continue

                # 找到foe_lane对应的交汇处的s
                for foe_lane_type_points_s_item_of_foe in foe_lane.foe_lane_type_points_s:
                    if foe_lane_type_points_s_item_of_foe[0] == current_lane.id:
                        foe_s = foe_lane_type_points_s_item_of_foe[3]
                        break

                # 在当前冲突lane中比较哪台foe车离给定的S最近，并且没有超出S，每条冲突lane只有一个候选冲突车
                delta = 10000  # 给一个极大值
                foe_car_block_road = False
                for foe_car_each in foe_cars:
                    # TODO 这一块应该改成计算两车间的距离

                    # foe_car_lane_id = SimPlatformAPI.myGetVehicleLane(foe_car_each)
                    foe_car_s_each = SimPlatformAPI.myGetDistanceFromLaneStart(foe_car_each)
                    total_s = foe_s + SimPlatformAPI.getVehicleWidth(
                        vehicle.vehicle_type) / 2 + SimPlatformAPI.myGetVehicleLength(foe_car_each)  # 车尾离开冲突区域总行驶距离
                    # 车头到foe点的距离
                    distance_from_head = abs(foe_s - foe_car_s_each)  # TODO 算距离这里还可以简化
                    # 车尾到foe点的距离
                    distance_from_end = abs(
                        foe_s - (foe_car_s_each - SimPlatformAPI.myGetVehicleLength(foe_car_each)))
                    min_distance = min(distance_from_head, distance_from_end)
                    if foe_car_s_each < total_s and min_distance < delta:
                        delta = min_distance
                        real_foe_car = foe_car_each
                        rest_foe_s = total_s - foe_car_s_each
                        if foe_car_s_each > foe_s:
                            foe_car_block_road = True
                        else:
                            foe_car_block_road = False

                if real_foe_car:
                    foe_vehicle = Dao.vehicle_dictionary[real_foe_car]
                    potential_foe_vehicles_inter_s.append(
                        (foe_vehicle, rest_foe_s, foe_car_block_road, foe_lane_type_points_s_item[3] - s))

            # 没有搜索到敌对车
            if not potential_foe_vehicles_inter_s:
                return cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)

            # 拿到了foe车之后
            else:
                for item in potential_foe_vehicles_inter_s:
                    log.info("{},{},{},{}".format(item[0].id, item[1], item[2], item[3]))
                    target_vehicle = item[0]
                    foe_rest_s = item[1]
                    foe_car_block_road = item[2]
                    rest_s = item[3]
                    return cls.get_and_update_accelerator_by_ST(vehicle, trajectory, rest_s, target_vehicle,
                                                                foe_rest_s, foe_car_block_road)

    @staticmethod
    def get_and_update_accelerator_by_ST(vehicle: Vehicle, trajectory: Trajectory, dis_to_intersection: float,
                                         foe_vehicle: Vehicle, foe_dis_to_intersection: float,
                                         foe_car_block_road: bool) -> float:

        veh_speed = vehicle.current_speed / 3.6
        foe_speed = foe_vehicle.current_speed / 3.6

        road_right = False

        if veh_speed <= 0:
            road_right = road_right
        elif foe_speed <= 0:
            road_right = True
        elif dis_to_intersection / veh_speed > foe_dis_to_intersection / foe_speed:
            road_right = True

        if road_right:
            acc = 0  # 保持匀速行驶
            return acc
        else:
            half_of_foe_car_length = SimPlatformAPI.myGetVehicleLength(foe_vehicle.id) / 2
            half_of_foe_car_width = SimPlatformAPI.getVehicleWidth(foe_vehicle.vehicle_type) / 2
            half_of_foe_car_diagonal_length = pow(half_of_foe_car_length ** 2 + half_of_foe_car_width ** 2, 1 / 2)
            t_2 = foe_dis_to_intersection / foe_vehicle.current_speed
            s = dis_to_intersection - half_of_foe_car_diagonal_length
            a = 2 * s / t_2 ** 2

            return a

    @staticmethod
    def get_and_update_accelerator_normal(vehicle, trajectory: Trajectory) -> int:
        """private方法，返回idm跟驰模型下的加速度"""
        # a_max = 2  # 最大加速度
        # v_desired = 70 / 3.6  # 汽车行驶期望速度
        a_max = vehicle.max_accel  # 最大加速度
        v_desired = min(80 / 3.6, vehicle.max_v / 3.6)  # 汽车行驶期望速度
        delta = 4  # 自由加速指数δ
        t_desired = 1.5  # 汽车安全车头时距T
        s0 = 3  # 最小安全间距
        b_comfortable = 2  # 期望舒适减速度

        current_speed = vehicle.current_speed  # 当前车速
        # current_speed = myGetVehicleSpeed(vehicle.id)
        if current_speed < 0:
            log.info("the current_speed is lower than 0,PLS check the logic or API")
            current_speed = 0

        """
        if current_speed < -5:
            current_speed = -5
            
        """

        leader = TrafficInterface.get_leader_vehicle(vehicle.id, trajectory)

        """以下是IDM模型公式计算"""

        if leader < -1:  # invalid，前方无路
            # print(str(vehicle.id) + "前方无路")
            acc = a_max * (1. - pow((current_speed / v_desired), delta))
        else:
            if leader == -1:  # 前方无车
                acc = a_max * (1. - pow((current_speed / v_desired), delta))
            else:
                try:  # 主车存在的情况下，超出接管范围的车不在dao中，如果出现key error，就用pano接口获取前车速度
                    leader_vehicle = Dao.vehicle_dictionary[leader]
                except:
                    s_delta = TrafficInterface.two_car_distance(vehicle.id, leader) - SimPlatformAPI.myGetVehicleLength(
                        leader)  # 车辆间距
                    if s_delta < 0:
                        s_delta = 0.001
                    v_delta = current_speed - SimPlatformAPI.myGetVehicleSpeed(leader)  # 前后车速度差绝对值
                    s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                                2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
                    acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))
                else:
                    s_delta = TrafficInterface.two_car_distance(vehicle.id, leader) - SimPlatformAPI.myGetVehicleLength(
                        leader)  # 车辆间距
                    if s_delta < 0:
                        s_delta = 0.001
                    v_delta = current_speed - leader_vehicle.current_speed  # 前后车速度差绝对值
                    s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                                2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
                    acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))

        return acc

    @staticmethod
    def get_and_update_accelerator_change_lane(vehicle, trajectory: Trajectory) -> int:

        a_max = vehicle.max_accel  # 最大加速度
        v_desired = vehicle.max_v / 2 / 3.6  # 汽车行驶期望速度
        delta = 4  # 自由加速指数δ
        t_desired = 1.5  # 汽车安全车头时距T
        s0 = 2  # 最小安全间距
        b_comfortable = 2  # 期望舒适减速度

        current_speed = vehicle.current_speed
        if current_speed < 0:
            log.warning("the current_speed is lower than 0,PLS check the logic or API")
            current_speed = 0

        """
        if current_speed < -5:
            current_speed = -5
            vehicle.current_speed = current_speed
        """

        leader = TrafficInterface.get_leader_vehicle(vehicle.id, trajectory)
        current_lane = SimPlatformAPI.myGetLaneID(vehicle.id)
        origin_lane = trajectory.continuous_lane_id[0]
        if leader <= -1:
            if vehicle.driving_mode == DrivingMode.RIGHT_CHANGING_LANE and current_lane == origin_lane:
                right_follower = SimPlatformAPI.myGetRightFollowerVehicle(vehicle.id)
                if right_follower > 0 and TrafficInterface.two_car_distance(vehicle.id,
                                                                            right_follower) < SimPlatformAPI.myGetVehicleLength(
                        vehicle.id) + s0:
                    v_delta = current_speed
                    s_delta = 0.1  # 这里应该是设为具体有多远，比如在目标车道上了就设为0.1，没在目标车道上就设大一点，避免车子看起来急刹车
                else:
                    acc = a_max * (1. - pow((current_speed / v_desired), delta))
                    return acc
            elif vehicle.driving_mode == DrivingMode.LEFT_CHANGING_LANE and current_lane == origin_lane:
                left_follower = SimPlatformAPI.myGetLeftFollowerVehicle(vehicle.id)
                if left_follower > 0 and TrafficInterface.two_car_distance(vehicle.id,
                                                                           left_follower) < SimPlatformAPI.myGetVehicleLength(
                        vehicle.id) + s0:
                    v_delta = current_speed
                    s_delta = 0.1
                else:
                    acc = a_max * (1. - pow((current_speed / v_desired), delta))
                    return acc
            else:
                acc = a_max * (1. - pow((current_speed / v_desired), delta))
                return acc
        else:
            # leader_vehicle=Dao.vehicle_dictionary[leader] 为了有主车，还是大地图的情况下，缩小范围所做
            if vehicle.driving_mode == DrivingMode.RIGHT_CHANGING_LANE and current_lane == origin_lane:
                right_follower = SimPlatformAPI.myGetRightFollowerVehicle(vehicle.id)
                if right_follower > 0 and TrafficInterface.two_car_distance(vehicle.id,
                                                                            right_follower) < SimPlatformAPI.myGetVehicleLength(
                        right_follower) + s0:
                    v_delta = current_speed
                    s_delta = 0.1
                else:
                    v_delta = current_speed - SimPlatformAPI.myGetVehicleSpeed(leader)  # 前后车速度差绝对值
                    s_delta = TrafficInterface.two_car_distance(vehicle.id, leader) - SimPlatformAPI.myGetVehicleLength(
                        leader)  # 车辆间距
            elif vehicle.driving_mode == DrivingMode.LEFT_CHANGING_LANE and current_lane == origin_lane:
                left_follower = SimPlatformAPI.myGetLeftFollowerVehicle(vehicle.id)
                if left_follower > 0 and TrafficInterface.two_car_distance(vehicle.id,
                                                                           left_follower) < SimPlatformAPI.myGetVehicleLength(
                        left_follower) + s0:
                    v_delta = current_speed
                    s_delta = 0.1
                else:
                    v_delta = current_speed - SimPlatformAPI.myGetVehicleSpeed(leader)  # 前后车速度差绝对值
                    s_delta = TrafficInterface.two_car_distance(vehicle.id, leader) - SimPlatformAPI.myGetVehicleLength(
                        leader)  # 车辆间距
            # 车头进入了目标车道了
            else:
                v_delta = current_speed - SimPlatformAPI.myGetVehicleSpeed(leader)  # 前后车速度差绝对值
                s_delta = TrafficInterface.two_car_distance(vehicle.id, leader) - SimPlatformAPI.myGetVehicleLength(
                    leader)  # 车辆间距
        # print("I'm:",vehicle.id,"leader is:",leader,"s_delta is:",s_delta)
        # print("two_car_distance is ",TrafficInterface.two_car_distance(vehicle.id, leader),"leader length is:",myGetVehicleLength(leader))
        if s_delta < 0.1:
            s_delta = 0.1
        s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
        acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))

        return acc

    # 临近路口或者车头前脸在路口内，又没有敌对车时的情况下。计算加速度
    @staticmethod
    def get_and_update_accelerator_normal_internal(vehicle: Vehicle, trajectory: Trajectory) -> float:
        """private方法，返回idm跟驰模型下的加速度"""
        # a_max = 2  # 最大加速度
        # v_desired = 70 / 3.6  # 汽车行驶期望速度
        a_max = vehicle.a_max  # 最大加速度
        v_desired = vehicle.v_desired  # 汽车行驶期望速度 ,路口内要够慢
        delta = vehicle.delta  # 自由加速指数δ
        t_desired = vehicle.t_desired  # 汽车安全车头时距T
        s0 = vehicle.s0  # 最小安全间距
        b_comfortable = vehicle.b_comfortable  # 期望舒适减速度

        # current_speed = vehicle.current_speed  # 当前车速
        current_speed = vehicle.current_speed
        if current_speed < 0:
            log.warning("the current_speed is lower than 0,PLS check the logic or API")
            current_speed = 0

        """
        if current_speed < -5:
            current_speed = -5
            vehicle.current_speed = current_speed
        """

        current_s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)  # 当前离起始点的距离
        # current_x = myGetVehicleX(vehicle.id) # 当前x坐标

        leader = TrafficInterface.get_leader_vehicle(vehicle.id, trajectory)
        # leader = SimPlatformAPI.myGetLeaderVehicle(vehicle.id)

        """以下是IDM模型公式计算"""

        if leader < -1 or leader == vehicle.id:  # invalid，前方无路,等于自己 是因为即将跨越停止线的瞬间，Pano提供的API返回的当前路和trajectory中记录的不一样，导致后续取前车取到了自己
            # print(str(vehicle.id) + "前方无路")
            acc = a_max * (1. - pow((current_speed / v_desired), delta))
        else:
            if leader == -1:  # 前方无车
                acc = a_max * (1. - pow((current_speed / v_desired), delta))
            else:
                try:  # 主车存在的情况下，超出接管范围的车不在dao中，如果出现key error，就用pano接口获取前车速度
                    leader_vehicle = Dao.vehicle_dictionary[leader]
                except:
                    s_delta = TrafficInterface.two_car_distance(vehicle.id, leader) - SimPlatformAPI.myGetVehicleLength(
                        leader)  # 车辆间距
                    if s_delta < 0:
                        s_delta = 0.001
                    v_delta = current_speed - SimPlatformAPI.myGetVehicleSpeed(leader)  # 前后车速度差绝对值
                    s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                                2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
                    acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))
                else:
                    s_delta = TrafficInterface.two_car_distance(vehicle.id, leader) - SimPlatformAPI.myGetVehicleLength(
                        leader)  # 车辆间距
                    if s_delta < 0:
                        s_delta = 0.001
                    v_delta = current_speed - leader_vehicle.current_speed  # 前后车速度差绝对值
                    s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                                2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
                    acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))

        return acc

    # 废弃
    # @staticmethod
    # def get_and_update_accelerator_internal(vehicle: Vehicle, for_car_id: int) -> int:
    #     foeDisToEnd = SimPlatformAPI.myGetDistanceToLaneEnd(for_car_id)
    #     myDisToEnd = SimPlatformAPI.myGetDistanceToLaneEnd(vehicle.id)
    #     foeArrivalTime = 0
    #     acc = 0
    #     if SimPlatformAPI.myGetVehicleSpeed(for_car_id) < 0.1:  # todo: 这里未来还需要考虑一下交通灯
    #         vehicle.set_speed(0)
    #         # myDebugPrint(id, "StopByFoe")
    #     else:
    #         foeArrivalTime = foeDisToEnd / myGetVehicleSpeed(for_car_id)
    #         foeLeaveTime = (foeDisToEnd + SimPlatformAPI.myGetVehicleLength(
    #             for_car_id) * 2 + 2) / SimPlatformAPI.myGetVehicleSpeed(for_car_id) + 1  # todo: *2, +2(s0), +1是做些保留
    #         acc = 2 * (myDisToEnd - vehicle.current_speed * foeLeaveTime) / (foeLeaveTime * foeLeaveTime)
    #
    #     vehicle.set_acceleration(acc)
    #     return acc

    # 车头前脸在路口内部，有敌对车的情况。车头前脸在停止线内，有敌对车的情况。暂时未使用
    @classmethod
    def get_and_update_accelerator_internal(cls, vehicle: Vehicle, trajectory: Trajectory, dis_to_intersection: float,
                                            foe_vehicle: Vehicle, foe_dis_to_intersection: float,
                                            foe_car_block_road: bool) -> float:

        # my_vehicle_length = SimPlatformAPI.myGetVehicleLength(vehicle.id)
        normal_acc = cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)
        if foe_car_block_road:
            """private方法，返回idm跟驰模型下的加速度"""
            a_max = vehicle.max_accel  # 最大加速度
            v_desired = vehicle.max_v / 1.5 / 3.6  # 汽车行驶期望速度 ,路口内要够慢
            delta = 4  # 自由加速指数δ
            t_desired = 1.5  # 汽车安全车头时距T
            s0 = 6  # 最小安全间距
            b_comfortable = 2  # 期望舒适减速度

            current_speed = vehicle.current_speed
            if current_speed < 0:
                log.warning("the current_speed is lower than 0,PLS check the logic or API")
                current_speed = 0

            leader_v = 0
            v_delta = current_speed - leader_v  # 前后车速度差绝对值
            s_delta = dis_to_intersection  # 车辆前脸正中心到交点的距离
            if s_delta < 0.1:
                s_delta = 0.1
            s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                    2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
            acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))

            if acc < normal_acc:
                return acc
            else:
                return normal_acc

        my_speed = vehicle.current_speed
        if my_speed <= 0:
            my_spend_time = 10000
        else:
            my_spend_time = (dis_to_intersection + SimPlatformAPI.myGetVehicleLength(foe_vehicle.id)) / my_speed
        # foe_vehicle_length = SimPlatformAPI.myGetVehicleLength(foe_vehicle.id)
        foe_speed = foe_vehicle.current_speed
        if foe_speed <= 0:
            foe_spend_time = 9999
        else:
            foe_spend_time = (foe_dis_to_intersection + SimPlatformAPI.myGetVehicleLength(foe_vehicle.id)) / foe_speed
        if my_spend_time <= foe_spend_time:
            # acc = 2*(dis_to_intersection)/(foe_spend_time*foe_spend_time)
            """private方法，返回idm跟驰模型下的加速度"""
            a_max = vehicle.max_accel  # 最大加速度
            v_desired = 40 / 3.6  # 汽车行驶期望速度 ,路口内要够慢
            delta = 4  # 自由加速指数δ
            current_speed = vehicle.current_speed
            if current_speed < 0:
                log.warning("the current_speed is lower than 0,PLS check the logic or API")
                current_speed = 0

            """以下是IDM模型公式计算"""
            # 我花的时间少，我就快速通过，使用IDM模型中没有前车情况下的公式
            acc = a_max * (1. - pow((current_speed / v_desired), delta))

            if acc < normal_acc:
                return acc
            else:
                return normal_acc
        if my_spend_time > foe_spend_time:
            """private方法，返回idm跟驰模型下的加速度"""
            a_max = 3  # 最大加速度
            v_desired = 20 / 3.6  # 汽车行驶期望速度 ,路口内要够慢
            delta = 4  # 自由加速指数δ
            t_desired = 1.5  # 汽车安全车头时距T
            s0 = 6  # 最小安全间距
            b_comfortable = 2  # 期望舒适减速度

            current_speed = vehicle.current_speed
            if current_speed < 0:
                log.warning("the current_speed is lower than 0,PLS check the logic or API")
                current_speed = 0

            leader_v = 0
            v_delta = current_speed - leader_v  # 前后车速度差绝对值
            s_delta = dis_to_intersection  # 车辆前脸正中心到交点的距离
            if s_delta < 0.1:
                s_delta = 0.1
            s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                    2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
            acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))

            if acc < normal_acc:
                return acc
            else:
                return normal_acc

    @classmethod
    def get_and_update_accelerator_internal_by_distance(cls, vehicle: Vehicle, trajectory: Trajectory,
                                                        dis_to_intersection: float,
                                                        foe_vehicle: Vehicle, foe_dis_to_intersection: float,
                                                        foe_car_block_road: bool) -> float:

        # my_vehicle_length = SimPlatformAPI.myGetVehicleLength(vehicle.id)
        normal_acc = cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)
        if foe_car_block_road:
            """private方法，返回idm跟驰模型下的加速度"""
            a_max = rng.integers(3, 5)  # 最大加速度
            v_desired = vehicle.max_v / 1.5 / 3.6  # 汽车行驶期望速度 ,路口内要够慢
            delta = 4  # 自由加速指数δ
            t_desired = 1.5  # 汽车安全车头时距T
            s0 = 0  # 最小安全间距
            b_comfortable = 2  # 期望舒适减速度

            current_speed = vehicle.current_speed
            if current_speed < 0:
                log.warning("the current_speed is lower than 0,PLS check the logic or API")
                current_speed = 0

            leader_v = 0
            v_delta = current_speed - leader_v  # 前后车速度差绝对值
            s_delta = dis_to_intersection - SimPlatformAPI.myGetVehicleLength(
                vehicle.id) - SimPlatformAPI.getVehicleWidth(foe_vehicle.vehicle_type) / 2 - 5  # 车辆前脸正中心到碰撞位置的距离
            if s_delta < 0.1:
                s_delta = 0.1
            s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                    2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
            acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))

            if acc < normal_acc:
                return acc
            else:
                return normal_acc

        if dis_to_intersection <= foe_dis_to_intersection:
            # acc = 2*(dis_to_intersection)/(foe_spend_time*foe_spend_time)
            """private方法，返回idm跟驰模型下的加速度"""
            a_max = rng.integers(3, 5)  # 最大加速度
            v_desired = vehicle.max_v / 1.5 / 3.6  # 汽车行驶期望速度 ,路口内要够慢
            delta = 4  # 自由加速指数δ
            current_speed = vehicle.current_speed
            if current_speed < 0:
                log.warning("the current_speed is lower than 0,PLS check the logic or API")
                current_speed = 0

            """以下是IDM模型公式计算"""
            # 我花的时间少，我就快速通过，使用IDM模型中没有前车情况下的公式
            acc = a_max * (1. - pow((current_speed / v_desired), delta))

            if acc < normal_acc:
                return acc
            else:
                return normal_acc
        if dis_to_intersection > foe_dis_to_intersection:
            """private方法，返回idm跟驰模型下的加速度"""
            a_max = rng.integers(3, 5)  # 最大加速度
            v_desired = vehicle.max_v / 1.5 / 3.6  # 汽车行驶期望速度 ,路口内要够慢
            delta = 4  # 自由加速指数δ
            t_desired = 1.5  # 汽车安全车头时距T
            s0 = 0  # 最小安全间距
            b_comfortable = 2  # 期望舒适减速度

            current_speed = vehicle.current_speed
            if current_speed < 0:
                log.warning("the current_speed is lower than 0,PLS check the logic or API")
                current_speed = 0

            leader_v = 0
            v_delta = current_speed - leader_v  # 前后车速度差绝对值
            s_delta = dis_to_intersection - SimPlatformAPI.getVehicleWidth(
                foe_vehicle.vehicle_type) / 2  # 车辆前脸正中心到交点的距离
            if s_delta < 0.1:
                s_delta = 0.1
            s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                    2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
            acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))

            if acc < normal_acc:
                return acc
            else:
                return normal_acc

    @classmethod
    def get_and_update_accelerator_internal_by_distance_new(cls, vehicle: Vehicle, trajectory: Trajectory,
                                                            dis_to_intersection: float,
                                                            foe_vehicle: Vehicle, foe_dis_to_intersection: float,
                                                            foe_car_block_road: bool) -> float:

        # my_vehicle_length = SimPlatformAPI.myGetVehicleLength(vehicle.id)
        normal_acc = cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)

        if dis_to_intersection <= foe_dis_to_intersection:
            # acc = 2*(dis_to_intersection)/(foe_spend_time*foe_spend_time)
            """private方法，返回idm跟驰模型下的加速度"""
            a_max = 5  # 最大加速度
            v_desired = 50 / 3.6  # 汽车行驶期望速度 ,路口内要够慢
            delta = 4  # 自由加速指数δ
            current_speed = vehicle.current_speed
            if current_speed < 0:
                log.warning("the current_speed is lower than 0,PLS check the logic or API")
                current_speed = 0

            """以下是IDM模型公式计算"""
            # 我花的时间少，我就快速通过，使用IDM模型中没有前车情况下的公式
            acc = a_max * (1. - pow((current_speed / v_desired), delta))

            if acc < normal_acc:
                return acc
            else:
                return normal_acc
        if dis_to_intersection > foe_dis_to_intersection:
            """private方法，返回idm跟驰模型下的加速度"""
            a_max = 5  # 最大加速度
            v_desired = 120 / 3.6  # 汽车行驶期望速度 ,路口内要够慢
            delta = 4  # 自由加速指数δ
            t_desired = 1.5  # 汽车安全车头时距T
            s0 = 1  # 最小安全间距
            b_comfortable = 6  # 期望舒适减速度

            current_speed = vehicle.current_speed
            if current_speed < 0:
                log.warning("the current_speed is lower than 0,PLS check the logic or API")
                current_speed = 0

            leader_v = foe_vehicle.current_speed
            v_delta = current_speed - leader_v  # 前后车速度差绝对值
            s_delta = dis_to_intersection - foe_dis_to_intersection - foe_vehicle.length
            if s_delta < 0.1:
                s_delta = 0.1
            s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                    2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
            acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))

            if acc < normal_acc:
                return acc
            else:
                return normal_acc

    @classmethod
    def get_and_update_accelerator_internal_with_foe(cls, vehicle: Vehicle, trajectory: Trajectory,
                                                     foe_vehicle: Vehicle, my_delta_s: float) -> float:

        # my_vehicle_length = SimPlatformAPI.myGetVehicleLength(vehicle.id)
        normal_acc = cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)
        # normal_acc=1000
        """private方法，返回idm跟驰模型下的加速度"""
        a_max = vehicle.a_max  # 最大加速度
        v_desired = vehicle.v_desired  # 汽车行驶期望速度 ,路口内要够慢
        delta = vehicle.delta  # 自由加速指数δ
        t_desired = vehicle.t_desired  # 汽车安全车头时距T
        s0 = vehicle.s0  # 最小安全间距
        b_comfortable = vehicle.b_comfortable  # 期望舒适减速度

        current_speed = vehicle.current_speed
        if current_speed < 0:
            log.warning("the current_speed is lower than 0,PLS check the logic or API")
            current_speed = 0

        leader_v = foe_vehicle.current_speed
        v_delta = current_speed - leader_v  # 前后车速度差绝对值

        s_delta = my_delta_s
        # if s_delta < 0.1:
        #     s_delta = 0.1
        s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
        acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))

        if acc < normal_acc:
            return acc
        else:
            return normal_acc

    # 在路口内，且有foe车
    @classmethod
    def get_and_update_accelerator_internal_by_time(cls, vehicle: Vehicle, trajectory: Trajectory,
                                                    dis_to_intersection: float,
                                                    foe_vehicle: Vehicle, foe_dis_to_intersection: float,
                                                    foe_car_block_road: bool) -> float:

        # my_vehicle_length = SimPlatformAPI.myGetVehicleLength(vehicle.id)
        normal_acc = cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)

        my_speed = vehicle.current_speed
        if my_speed <= 0:
            my_spend_time = 10000
        else:
            my_spend_time = (dis_to_intersection) / my_speed
        # foe_vehicle_length = SimPlatformAPI.myGetVehicleLength(foe_vehicle.id)
        foe_speed = foe_vehicle.current_speed
        if foe_speed <= 0:
            foe_spend_time = 9999
        else:
            foe_spend_time = (foe_dis_to_intersection) / foe_speed
        if my_spend_time <= foe_spend_time:
            # acc = 2*(dis_to_intersection)/(foe_spend_time*foe_spend_time)
            """private方法，返回idm跟驰模型下的加速度"""
            a_max = 5  # 最大加速度
            v_desired = 50 / 3.6  # 汽车行驶期望速度 ,路口内要够慢
            delta = 4  # 自由加速指数δ
            current_speed = vehicle.current_speed
            if current_speed < 0:
                log.warning("the current_speed is lower than 0,PLS check the logic or API")
                current_speed = 0

            """以下是IDM模型公式计算"""
            # 我花的时间少，我就快速通过，使用IDM模型中没有前车情况下的公式
            acc = a_max * (1. - pow((current_speed / v_desired), delta))

            if acc < normal_acc:
                return acc
            else:
                return normal_acc
        if my_spend_time > foe_spend_time:
            """private方法，返回idm跟驰模型下的加速度"""
            a_max = 5  # 最大加速度
            v_desired = 50 / 3.6  # 汽车行驶期望速度 ,路口内要够慢
            delta = 4  # 自由加速指数δ
            t_desired = 1.5  # 汽车安全车头时距T
            s0 = 6  # 最小安全间距
            b_comfortable = 6  # 期望舒适减速度

            current_speed = vehicle.current_speed
            if current_speed < 0:
                log.warning("the current_speed is lower than 0,PLS check the logic or API")
                current_speed = 0

            leader_v = foe_vehicle.current_speed
            v_delta = current_speed - leader_v  # 前后车速度差绝对值
            s_delta = dis_to_intersection - foe_dis_to_intersection - foe_vehicle.length
            if s_delta < 0:
                s_delta = 0.1
            s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                    2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
            acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))

            if acc < normal_acc:
                return acc
            else:
                return normal_acc

    # 红灯的情况下
    @classmethod
    def get_and_update_accelerator_stop_line(cls, vehicle: Vehicle, dis_to_stop_line: float,
                                             trajectory: Trajectory) -> float:

        # original_acc=SimPlatformAPI.myGetVehicleAccel(vehicle.id)
        normal_acc = cls.get_and_update_accelerator_normal_internal(vehicle, trajectory)

        a_max = 3  # 最大加速度
        v_desired = vehicle.max_v / 2 / 3.6  # 汽车行驶期望速度 ,路口内要够慢
        delta = 4  # 自由加速指数δ
        t_desired = 1.5  # 汽车安全车头时距T
        s0 = 1  # 最小安全间距
        b_comfortable = 2  # 期望舒适减速度

        current_speed = vehicle.current_speed
        if current_speed < 0:
            log.warning("the current_speed is lower than 0,PLS check the logic or API")
            current_speed = 0

        leader_v = 0
        v_delta = current_speed - leader_v  # 前后车速度差绝对值
        s_delta = dis_to_stop_line
        if s_delta < 0.1:
            s_delta = 0.1
        s_star = s0 + current_speed * t_desired + current_speed * v_delta / (
                2. * pow(a_max * b_comfortable, 0.5))  # 理想期望间距
        acc = a_max * (1. - pow((current_speed / v_desired), delta) - pow(s_star / s_delta, 2.))

        # print("我的前车是：", leader, "normal acc is:", normal_acc,"acc is:",acc)
        if acc < normal_acc:
            return acc
        else:
            return normal_acc

    @classmethod
    def generate_trajectory_point_set(cls, vehicle: Vehicle, trajectory: Trajectory, MultiThreadPool):
        """根据vehicle.driving_mode来选择路径的更新方式，默认为idm方式"""
        log.info("last time vehicle.driving_mode is:{}".format(vehicle.driving_mode))
        # 特殊逻辑，应对3车道变两车道
        # current_lane_id = SimPlatformAPI.myGetVehicleLane(vehicle.id)
        # current_lane = Dao.lane_dictionary[current_lane_id]

        if vehicle.driving_mode is None:
            vehicle.set_driving_mode(DrivingMode.FOLLOW_DRIVING)

        # # 特殊逻辑，应对3车道变两车道
        # elif not SimPlatformAPI.myIsDeadEnd(current_lane_id) and not current_lane.next_lane:
        #     vehicle.set_driving_mode(DrivingMode.RIGHT_CHANGING_LANE)

        # 强制换道
        elif vehicle.located_area == LocatedArea.FORCE_CHANGE_TO_LEFT:
            vehicle.set_driving_mode(DrivingMode.LEFT_CHANGING_LANE)
        elif vehicle.located_area == LocatedArea.FORCE_CHANGE_TO_RIGHT:
            vehicle.set_driving_mode(DrivingMode.RIGHT_CHANGING_LANE)

        # 换道需求判断
        elif vehicle.driving_mode == DrivingMode.FOLLOW_DRIVING:
            if vehicle.located_area != LocatedArea.ADJACENT_JUNCTION_AREA:
                random_value = rng.integers(0, 9)
                if random_value > 1:
                    direction = TrafficInterface.get_free_change_lane_direction(vehicle.id)
                    if direction is not None:
                        if direction == Direction.LEFT:
                            vehicle.set_driving_mode(DrivingMode.LEFT_CHANGING_LANE)
                        elif direction == Direction.RIGHT:
                            vehicle.set_driving_mode(DrivingMode.RIGHT_CHANGING_LANE)
                    else:
                        log.warning("get_free_change_lane_direction return None,keep FOLLOW_DRIVING")

        # 之前换过道了就别换了
        elif vehicle.driving_mode == DrivingMode.LEFT_CHANGING_LANE or vehicle.driving_mode == DrivingMode.RIGHT_CHANGING_LANE:
            vehicle.set_driving_mode(DrivingMode.FOLLOW_DRIVING)

        log.info("now vehicle.driving_mode is:{}".format(vehicle.driving_mode))

        # # IDM pybind11 多线程模式
        # cls.generate_point_set_track_by_multi_thread(vehicle, trajectory,MultiThreadPool)

        # 只有IDM 单线程模式
        # cls.generate_point_set_track(vehicle, trajectory)

        # IDM+换道逻辑
        if vehicle.driving_mode == DrivingMode.FOLLOW_DRIVING:
            cls.generate_point_set_track_by_multi_thread(vehicle, trajectory, MultiThreadPool)
        elif vehicle.driving_mode == DrivingMode.LEFT_CHANGING_LANE:
            cls.generate_point_set_waypoint_by_multi_thread(vehicle, Direction.LEFT, MultiThreadPool)
        elif vehicle.driving_mode == DrivingMode.RIGHT_CHANGING_LANE:
            cls.generate_point_set_waypoint_by_multi_thread(vehicle, Direction.RIGHT, MultiThreadPool)

        """
        # switch的替代方案

        return {
            DrivingMode.FOLLOW_DRIVING: cls.generate_point_set_idm(vehicle),
            DrivingMode.LEFT_CHANGING_LANE: cls.generate_point_set_waypoint(vehicle,Direction.LEFT),
            DrivingMode.RIGHT_CHANGING_LANE: cls.generate_point_set_waypoint(vehicle, Direction.RIGHT)
        }.get(vehicle.driving_mode)
        """

    '''
    @classmethod
    def generate_point_set_idm(cls, vehicle):
        """
        对vehicle的trajectory生成路径
        :param vehicle:
        :return:
        """

        print("begin to generate_point_set_idm")
        trajectory = Dao.get_trajectory_by_id(vehicle.id)
        # 开始计算每个仿真步的坐标
        # 变量初始化
        current_lane_id = myGetLaneID(vehicle.id)
        cur_lane_max_s = TrafficInterface.myGetLaneLength(current_lane_id)
        # cur_lane_max_s=SimPlatformAPI.myGetLaneLength(current_lane_id)
        # print('----cur_lane_max_s_old is ', cur_lane_max_s_old, '------')
        # print('----cur_lane_max_s is ',cur_lane_max_s,'------')
        is_dead_end = SimPlatformAPI.myIsDeadEnd(current_lane_id)
        s = myGetDistanceFromLaneStart(vehicle.id)  # 此处减去1是因为有时候算出来的s比cur_lane_max_s还大
        if s > cur_lane_max_s:
            s = cur_lane_max_s

        if is_dead_end:
            print("dead end lane s:" + str(s))
            if s < 0:
                s = 0

        #  while begin
        lane_to_go = current_lane_id  # calculate_lane變了，lane_to_go才會變
        to_go_lane_max_s = cur_lane_max_s
        forecast_time = 0
        sl_list = []  # 当前lane上的sl
        x_y_s_yaw_set = []
        gernarate_finished = False
        # 进入计算每个时间间隔路径数据的循环
        if is_dead_end:
            while forecast_time < trajectory.forecast_time_range:
                s = s + 40 * 0.01
                if s > to_go_lane_max_s:
                    break
                (x, y) = TrafficInterface.getXYFromSL(current_lane_id, s, 0)
                if x == None or y == None:
                    my_error = ValueError('x or y is none in current dead end lane')
                    raise my_error
                sl_list.append((x, y))
                forecast_time = forecast_time + 0.01  # 10ms仿真步

            if len(x_y_s_yaw_set) == 1:
                print("len of x_y_s_yaw_set is 1")
            elif len(x_y_s_yaw_set) > 1:
                x_y_s_yaw_set = AlgorithmInterface.getLaneShapeWithOnlyXY(sl_list)


        else:  # 不是dead end
            x_y_s_yaw_set_temp = []
            accumulate_s = 0  # 用于迭代计算下一条路用
            select_next_lane = False
            while forecast_time < trajectory.forecast_time_range:
                s = s + 40 * 0.01
                if s > to_go_lane_max_s:
                    """
                    是时候把下一条lane选进来了
                    """
                    accumulate_s = accumulate_s + to_go_lane_max_s
                    if len(sl_list) > 3:  # 没有值就不计算x，y，s，yaw了
                        x_y_s_yaw_set_temp = AlgorithmInterface.getLaneShapeWithOnlyXY(sl_list)
                    # accumulate_s = accumulate_s + to_go_lane_max_s

                    if select_next_lane:  # todo 这里判断select_next_lane是T还是F可以考虑删去
                        for item in x_y_s_yaw_set_temp:  # 由于无法直接修改元组，只能转换成list后来修改
                            item_list = list(item)
                            item_list[2] = item_list[2] + accumulate_s
                            x_y_s_yaw_set.append(tuple(item_list))

                    else:
                        x_y_s_yaw_set = x_y_s_yaw_set + x_y_s_yaw_set_temp

                    if (Dao.lane_dictionary[lane_to_go].is_dead_end == True):
                        break
                    print('current lane to go', lane_to_go)
                    lane_to_go = Dao.lane_dictionary[lane_to_go].next_lane[0]  # todo 后期把这里的0改成随机数

                    to_go_lane_max_s = TrafficInterface.myGetLaneLength(lane_to_go)
                    x_y_s_yaw_set_temp = []  # 重置Temp
                    s = 0  # 重置s
                    sl_list = []  # 清空sl_list
                    select_next_lane = True
                    continue

                (x, y) = TrafficInterface.getXYFromSL(lane_to_go, s, 0)
                if x == None or y == None:
                    my_error = ValueError('x or y is none in current lane')
                    raise my_error
                sl_list.append((x, y, yaw, cumS, lane_to_go, s, l))
                forecast_time = forecast_time + 0.01  # 10ms仿真步
            if not x_y_s_yaw_set:  # 没有值 只计算了车脚下的这条路
                x_y_s_yaw_set = AlgorithmInterface.getLaneShapeWithOnlyXY(sl_list)
        # while end

        # 得到结果了
        if not x_y_s_yaw_set:
            print('x_y_s_yaw_set is empty')
        else:
            trajectory.x_y_s_yaw_set = x_y_s_yaw_set
            # 把上一行生成的set中的s单独拿出老生成一个list
            trajectory.s_in_current_trajectory = []
            for item in x_y_s_yaw_set:
                trajectory.s_in_current_trajectory.append(item[2])  # 这里的2代表 x,y,s,yaw 中s的下标
        print("generate_point_set_idm end")
        return
    '''  # 陈旧的generate_point_set_idm

    @classmethod
    def generate_point_set_track(cls, vehicle, trajectory):
        log.info("begin to generate_point_set_track")

        # 开始计算每个仿真步的坐标
        # 变量初始化
        current_lane_id = SimPlatformAPI.myGetLaneID(vehicle.id)
        cur_lane_max_s = TrafficInterface.myGetLaneLength(current_lane_id)
        is_dead_end = SimPlatformAPI.myIsDeadEnd(current_lane_id)
        s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
        if s > cur_lane_max_s:
            s = cur_lane_max_s
        elif s < 0:
            s = 0

        #  while begin
        lane_to_go = current_lane_id  # calculate_lane變了，lane_to_go才會變
        to_go_lane_max_s = cur_lane_max_s
        forecast_time = 0
        x_y_laneid_s_l_cums_cuml_yaw_set = []
        # 进入计算每个时间间隔路径数据的循环
        if is_dead_end:
            while forecast_time < trajectory.forecast_time_range:
                if s > to_go_lane_max_s:
                    break
                (x, y) = TrafficInterface.getXYFromSL(lane_to_go, s, 0)
                if x == None or y == None:
                    my_error = ValueError('x or y is none in current dead end lane')
                    raise my_error

                x_y_laneid_s_l_cums_cuml_yaw_set.append((x, y, lane_to_go, s, 0, s, 0))
                s = s + 20 * 0.01
                forecast_time = forecast_time + 0.01  # 10ms仿真步

            if len(x_y_laneid_s_l_cums_cuml_yaw_set) == 1:
                log.info("len of x_y_laneid_s_l_yaw_cums_cuml is 1,error")
                return
            else:
                x_y_laneid_s_l_cums_cuml_yaw_set = \
                    AlgorithmInterface.get_yaw_in_generating_points(x_y_laneid_s_l_cums_cuml_yaw_set)

        else:  # 不是dead end
            accumulate_s = 0  # 用于迭代计算下一条路用
            while forecast_time < trajectory.forecast_time_range:
                if s > to_go_lane_max_s:
                    """
                    是时候把下一条lane选进来了
                    """
                    s = s - to_go_lane_max_s
                    accumulate_s = accumulate_s + to_go_lane_max_s
                    if (Dao.lane_dictionary[lane_to_go].is_dead_end == True):
                        break  # dead_end路就不用往下选了
                    log.info('current lane to go {}'.format(lane_to_go))
                    lane_to_go = Dao.lane_dictionary[lane_to_go].next_lane[0]  # todo 后期把这里的0改成随机数
                    to_go_lane_max_s = TrafficInterface.myGetLaneLength(lane_to_go)
                    # to_go_lane_max_s = SimPlatformAPI.myGetLaneLength(lane_to_go)

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
                s = s + 20 * 0.01
                forecast_time = forecast_time + 0.01  # 10ms仿真步

            x_y_laneid_s_l_cums_cuml_yaw_set = AlgorithmInterface.get_yaw_in_generating_points(
                x_y_laneid_s_l_cums_cuml_yaw_set)
        # while end

        # 得到结果了
        if not x_y_laneid_s_l_cums_cuml_yaw_set:
            log.info('x_y_laneid_s_l_cums_cuml_yaw_set is empty')
        else:
            trajectory.x_y_laneid_s_l_cums_cuml_yaw_set = x_y_laneid_s_l_cums_cuml_yaw_set
            # 把上一行生成的set中的s单独拿出老生成一个list
            trajectory.s_in_current_trajectory = []
            for item in x_y_laneid_s_l_cums_cuml_yaw_set:
                trajectory.s_in_current_trajectory.append(item[5])  # 这里的5代表x_y_laneid_s_l_cums_cuml中cums的下标
        log.info("generate_point_set_idm end")

    @classmethod
    def generate_point_set_track_by_multi_thread(cls, vehicle, trajectory, MultiThreadPool):
        log.info("begin to generate_point_set_track")
        # 获取当前道路的接下来几个路，生成continuous_laneid_length continuous_laneid_shape

        if len(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set) > 0:
            cur_lane_id = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[-1][2]
            connecting_flag = True
            s = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[-1][3]
        else:
            cur_lane_id = SimPlatformAPI.myGetLaneID(vehicle.id)
            connecting_flag = False
            s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
        cur_lane_max_s = TrafficInterface.myGetLaneLength(cur_lane_id)
        is_dead_end = SimPlatformAPI.myIsDeadEnd(cur_lane_id)
        to_go_lane_max_s = cur_lane_max_s

        if s > cur_lane_max_s:
            s = cur_lane_max_s
        elif s < 0:
            s = 0

        # 在dead_end lane的尽头就不管了,这个是在Pano中才这样做
        if is_dead_end and (cur_lane_max_s - s) < 10:  # TODO 和update_trajectory中的检查重复了
            trajectory.calculate_done = True
            return

        continuous_laneid = []  # 放入计算中使用的
        continuous_length = []
        continuous_shape = []
        route_edges = []
        lane_to_go = cur_lane_id
        vehicle_length = SimPlatformAPI.myGetVehicleLength(vehicle.id)
        current_continuous_laneid = trajectory.continuous_lane_id

        # length_of_continuous_lane_id=len(trajectory.continuous_lane_id)
        # index_of_first_laneId_in_set=trajectory.continuous_lane_id.index(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[-1][2])
        # if index_of_first_laneId_in_set<length_of_continuous_lane_id-1:
        #     current_continuous_laneid
        #     for num in range(0,index_of_first_laneId_in_set):
        #         continuous_laneid.append(trajectory.continuous_lane_id[num])
        #         continuous_length.append(TrafficInterface.myGetLaneLength(trajectory.continuous_lane_id[num]))
        #         continuous_shape.append(Dao.dicLaneShapes[trajectory.continuous_lane_id[num]])

        # if myGetLaneID(vehicle.id) != cur_lane_id:
        #     current_continuous_lane_id.append(myGetLaneID(vehicle.id))

        for i in range(0, 4):  # 准备当前道路以及其余4条lane
            if SimPlatformAPI.API_Name=='SumoAPI' and not SimPlatformAPI.myCheckInternalLane(lane_to_go):
                edge=SimPlatformAPI.myGetEdgeIDByLaneID(lane_to_go)
                route_edges.append(edge)
            continuous_laneid.append(lane_to_go)
            continuous_length.append(TrafficInterface.myGetLaneLength(lane_to_go))
            continuous_shape.append(Dao.dicLaneShapes[lane_to_go])
            if (Dao.lane_dictionary[lane_to_go].is_dead_end == True):
                log.info("{} is dead end".format(lane_to_go))
                break  # dead_end路就不用往下选了

            # lane_to_go = Dao.lane_dictionary[lane_to_go].next_lane[0]  # 这个地方的0考虑改成随机数
            lane_to_go = rng.choice(Dao.lane_dictionary[lane_to_go].next_lane)

            # if(lane_to_go=='gneE5_0'):
            #     print('gneE5_0 shape are ',continuous_laneid_shape[lane_to_go])
        if connecting_flag == True:
            current_continuous_laneid.extend(continuous_laneid)
            index = current_continuous_laneid.index(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0][2])
            if index == len(current_continuous_laneid) - 1:
                pass
            else:
                current_continuous_laneid = current_continuous_laneid[index:-1]
        else:
            current_continuous_laneid.extend(continuous_laneid)

        # continuous_lanes_and_length生成完毕
        # print(vehicle.id,"continuous_laneid are: ", continuous_laneid)
        # print(vehicle.id,"continuous_length are: ",continuous_length)
        # print(vehicle.id,"continuous_shape are: ",continuous_shape)
        log.info("this car {} going to calculate track in multithread".format(vehicle.id))
        if SimPlatformAPI.API_Name == 'SumoAPI':
            SimPlatformAPI.mySetRoute(trajectory.VehID, route_edges)
        MultiThreadPool.calculate_track(trajectory,
                                        connecting_flag,
                                        vehicle_length,
                                        cur_lane_id,
                                        cur_lane_max_s,
                                        is_dead_end,
                                        s,
                                        continuous_laneid,
                                        continuous_length,
                                        continuous_shape,
                                        current_continuous_laneid)

    @classmethod
    def pop_point_in_trajectory(cls, vehicle, trajectory,delta_T):
        """根据当前仿真步车辆的速度和在轨迹中的s找到应该更新的点,并删除过期的点"""
        acc = cls.get_and_update_accelerator(vehicle, trajectory)
        # if vehicle.acc_used_account < 10 and vehicle.acc_used_account > 0:
        #     acc = vehicle.current_acceleration
        #     vehicle.acc_used_account += 1
        # else:
        #     acc = cls.get_and_update_accelerator(vehicle, trajectory)
        #     vehicle.acc_used_account = 1

        if acc < -8:
            acc = -8
        elif acc > 8:
            acc = 8

        # vehicle.set_acceleration(acc)
        vehicle.going_to_update_acc = acc
        # print("vehicle.current_speed is",vehicle.current_speed)
        # print("API :vehicle.speed is", SimPlatformAPI.myGetVehicleSpeed(vehicle.id))
        if SimPlatformAPI.API_Name=='PanoAPI':
            dt = delta_T / 1000 #delta_T 单位ms
        elif SimPlatformAPI.API_Name=='SumoAPI':
            dt = delta_T # delta_T 单位s
        elif SimPlatformAPI.API_Name=='InteractionAPI':
            dt = delta_T / 1000

        result_speed = vehicle.current_speed + acc * dt
        # TODO result_speed需要有个限制，参数范围限制
        if result_speed < 0:
            result_speed = 0
        elif result_speed > 120 / 3.6:
            result_speed = 120 / 3.6
        # vehicle.set_speed(result_speed)
        vehicle.going_to_update_speed = result_speed

        # 如果汽车坐标发生跳变就应该更新车辆的轨迹
        """
        if abs(SimPlatformAPI.myGetVehicleX(vehicle.id) - trajectory.x_y_s_yaw_set[0][0]) > 20 or \
                abs(SimPlatformAPI.myGetVehicleY(vehicle.id) - trajectory.x_y_s_yaw_set[0][0]) > 20:
            vehicle.driving_mode_last_time_changed = True
            trajectory.delete_trajectory_data()
            return (None, None, None, None)
        """

        s = trajectory.s_in_current_trajectory[0] + vehicle.going_to_update_speed * dt  # 索引0表示车辆的当前位置

        continue_lane_length = dict()
        for item in trajectory.current_continuous_lane_id:
            continue_lane_length[item] = TrafficInterface.myGetLaneLength(item)

        try:
            result, pos = AlgorithmInterface.get_tuple_by_linear_interpolation(trajectory, s, continue_lane_length)
        except Exception as e:
            log.info('some Exception caught in get_tuple_by_linear_interpolation {}', e)
            raise e
        else:
            # print(result[0:8])
            # # print(pos)
            # print("pop_point_in_trajectory index:" + str(pos))
            # print("the len of set is:" + str(len(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set)))

            # 删除trajectory两个与轨迹相关列表中过期的值
            if pos == 0:
                trajectory.s_in_current_trajectory[0] = result[5]
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0] = result
            else:
                for count in range(0, pos):
                    trajectory.s_in_current_trajectory.popleft()
                    trajectory.x_y_laneid_s_l_cums_cuml_yaw_set.popleft()

                trajectory.s_in_current_trajectory[0] = result[5]
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0] = result
                # #todo 这个del操作似乎相当费时间！！！！！！！！！
                # del trajectory.s_in_current_trajectory[0:pos]
                # del trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0:pos]

            # for i in range(0, pos):
            #     del trajectory.s_in_current_trajectory[0]
            #     del trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0]
            # print('here')
            return result[0:8]

    # 换道轨迹规划
    @staticmethod
    def generate_point_set_waypoint(vehicle: Vehicle, direction):  # todo 需要引入车型来决定换道长度
        log.info('begin to generate_point_set_waypoint {}', direction)
        trajectory = Dao.get_trajectory_by_id(vehicle.id)
        # 开始计算换道轨迹每个仿真步的坐标
        # 变量初始化
        current_lane = SimPlatformAPI.myGetLaneID(vehicle.id)
        cur_lane_max_s = TrafficInterface.myGetLaneLength(current_lane)
        s_from_lane_start = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
        if s_from_lane_start > cur_lane_max_s:
            log.info('DistanceFromLaneStart > cur_lane_max_s')
            s_from_lane_start = cur_lane_max_s

        #  while begin
        calculate_lane = current_lane  # calculate_lane變了，lane_to_go才會變
        sl_list = []  # 当前lane上的sl
        x_y_laneid_s_l_cums_cuml_yaw_set = []
        distance = (10 + vehicle.current_speed) * trajectory.change_lane_needed_time  # 这里加10是避免车速为0的时候distance也为0
        lane_width = 3.2  # todo 道路宽度后期改成由接口获取
        s = 0
        if direction == Direction.LEFT:
            while s < distance:
                l = AlgorithmInterface.get_waypoint(s, distance, lane_width)
                if l > 1.6:  # 如果是向左换道超过了车道线
                    calculate_lane = SimPlatformAPI.myGetLeftLaneID(vehicle.id)
                    l = l - lane_width
                (x, y) = TrafficInterface.getXYFromSL(calculate_lane, s + s_from_lane_start, l)
                if x is not None and y is not None:
                    x_y_laneid_s_l_cums_cuml_yaw_set.append(
                        (x, y, calculate_lane, s + s_from_lane_start, l, s + s_from_lane_start, l))
                s = s + 10 * 0.01

        elif direction == Direction.RIGHT:
            while s < distance:
                l = AlgorithmInterface.get_waypoint(s, distance, -1 * lane_width)
                if l < -1.6:  # 如果是向左换道超过了车道线
                    calculate_lane = SimPlatformAPI.myGetRightLaneID(vehicle.id)
                    l = l + lane_width
                (x, y) = TrafficInterface.getXYFromSL(calculate_lane, s + s_from_lane_start, l)
                if x is not None and y is not None:
                    x_y_laneid_s_l_cums_cuml_yaw_set.append(
                        (x, y, calculate_lane, s + s_from_lane_start, l, s + s_from_lane_start, l))
                s = s + 10 * 0.01
        # while end
        x_y_laneid_s_l_cums_cuml_yaw_set = AlgorithmInterface.get_yaw_in_generating_points(
            x_y_laneid_s_l_cums_cuml_yaw_set)
        # 得到结果了
        trajectory.x_y_laneid_s_l_cums_cuml_yaw_set = x_y_laneid_s_l_cums_cuml_yaw_set
        # 把上一行生成的set中的s单独拿出老生成一个list
        trajectory.s_in_current_trajectory = []
        for item in x_y_laneid_s_l_cums_cuml_yaw_set:
            trajectory.s_in_current_trajectory.append(item[5])  # 这里的5代表x_y_laneid_s_l_cums_cuml中cums的下标
        log.info("generate_point_set_waypoint end {}".format(direction))
        return

    # 换道轨迹规划多线程版本
    @staticmethod
    def generate_point_set_waypoint_by_multi_thread(vehicle: Vehicle, direction, MultiThreadPool):  # todo 需要引入车型来决定换道长度
        trajectory = Dao.get_trajectory_by_id(vehicle.id)
        trajectory.current_continuous_lane_id = []
        # 开始计算换道轨迹每个仿真步的坐标
        # 变量初始化

        # if len(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set)>0:
        #     current_lane = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[-1][2]
        #     connecting_flag = True
        #     s_from_lane_start=trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[-1][3]
        # else:
        #     current_lane = myGetLaneID(vehicle.id)
        #     connecting_flag = False
        #     s_from_lane_start = myGetDistanceFromLaneStart(vehicle.id)
        current_lane = SimPlatformAPI.myGetLaneID(vehicle.id)
        s_from_lane_start = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle.id)
        cur_lane_max_s = TrafficInterface.myGetLaneLength(current_lane)
        if s_from_lane_start > cur_lane_max_s:
            log.info("DistanceFromLaneStart > cur_lane_max_s")
            s_from_lane_start = cur_lane_max_s

        distance = (10 + vehicle.current_speed) * trajectory.change_lane_needed_time  # 这里加10是避免车速为0的时候distance也为0
        if distance > (cur_lane_max_s - s_from_lane_start):
            distance = cur_lane_max_s - s_from_lane_start - 5

        if distance >= 40:
            distance = 40  # 60以内是近路口区域
        lane_width = 3.2  # todo 道路宽度后期改成由接口获取
        continuous_laneid = {}
        continuous_laneid["Middle"] = []
        continuous_laneid["Left"] = []
        continuous_laneid["Right"] = []
        continuous_length = {}
        continuous_shape = {}
        direction_int = 0

        if SimPlatformAPI.myIsDeadEnd(current_lane) or not Dao.lane_dictionary[current_lane].next_lane:
            if direction == Direction.LEFT:
                direction_int = 1
                # 当前路
                continuous_laneid["Middle"].append(current_lane)
                continuous_length[current_lane] = TrafficInterface.myGetLaneLength(current_lane)
                continuous_shape[current_lane] = Dao.dicLaneShapes[current_lane]

                # 左边的路
                left_lane = SimPlatformAPI.myGetLeftLaneID(vehicle.id)
                continuous_laneid["Left"].append(left_lane)
                continuous_length[left_lane] = TrafficInterface.myGetLaneLength(left_lane)
                continuous_shape[left_lane] = Dao.dicLaneShapes[left_lane]

            elif direction == Direction.RIGHT:
                direction_int = 2
                # 当前路
                continuous_laneid["Middle"].append(current_lane)
                continuous_length[current_lane] = TrafficInterface.myGetLaneLength(current_lane)
                continuous_shape[current_lane] = Dao.dicLaneShapes[current_lane]
                # 右边的路
                right_lane = SimPlatformAPI.myGetRightLaneID(vehicle.id)
                continuous_laneid["Right"].append(right_lane)
                continuous_length[right_lane] = TrafficInterface.myGetLaneLength(right_lane)
                continuous_shape[right_lane] = Dao.dicLaneShapes[right_lane]

        else:
            next_lane = Dao.lane_dictionary[current_lane].next_lane[0]

            if SimPlatformAPI.myIsDeadEnd(next_lane):  # 下一条路才是dead_end路
                if direction == Direction.LEFT:
                    direction_int = 1
                    # 当前路
                    continuous_laneid["Middle"].append(current_lane)
                    continuous_length[current_lane] = TrafficInterface.myGetLaneLength(current_lane)
                    continuous_shape[current_lane] = Dao.dicLaneShapes[current_lane]
                    # 下一条路
                    next_lane = Dao.lane_dictionary[current_lane].next_lane[0]
                    continuous_laneid["Middle"].append(next_lane)
                    continuous_length[next_lane] = TrafficInterface.myGetLaneLength(next_lane)
                    continuous_shape[next_lane] = Dao.dicLaneShapes[next_lane]

                    # 左边的路
                    left_lane = SimPlatformAPI.myGetLeftLaneID(vehicle.id)
                    continuous_laneid["Left"].append(left_lane)
                    continuous_length[left_lane] = TrafficInterface.myGetLaneLength(left_lane)
                    continuous_shape[left_lane] = Dao.dicLaneShapes[left_lane]
                    # 左边的路的下一条路
                    try:
                        left_next_lane = Dao.lane_dictionary[left_lane].next_lane[0]
                    except:
                        log.info('left_lane has no next lane')
                    else:
                        continuous_laneid["Left"].append(left_next_lane)
                        continuous_length[left_next_lane] = TrafficInterface.myGetLaneLength(left_next_lane)
                        continuous_shape[left_next_lane] = Dao.dicLaneShapes[left_next_lane]


                elif direction == Direction.RIGHT:
                    direction_int = 2
                    # 当前路
                    continuous_laneid["Middle"].append(current_lane)
                    continuous_length[current_lane] = TrafficInterface.myGetLaneLength(current_lane)
                    continuous_shape[current_lane] = Dao.dicLaneShapes[current_lane]
                    # 下一条路
                    next_lane = Dao.lane_dictionary[current_lane].next_lane[0]
                    continuous_laneid["Middle"].append(next_lane)
                    continuous_length[next_lane] = TrafficInterface.myGetLaneLength(next_lane)
                    continuous_shape[next_lane] = Dao.dicLaneShapes[next_lane]

                    # 右边的路
                    right_lane = SimPlatformAPI.myGetRightLaneID(vehicle.id)
                    continuous_laneid["Right"].append(right_lane)
                    continuous_length[right_lane] = TrafficInterface.myGetLaneLength(right_lane)
                    continuous_shape[right_lane] = Dao.dicLaneShapes[right_lane]
                    # 右边的路的下一条路
                    try:
                        right_next_lane = Dao.lane_dictionary[right_lane].next_lane[0]
                    except:
                        log.info('right_lane has no next lane')
                    else:
                        continuous_laneid["Right"].append(right_next_lane)
                        continuous_length[right_next_lane] = TrafficInterface.myGetLaneLength(right_next_lane)
                        continuous_shape[right_next_lane] = Dao.dicLaneShapes[right_next_lane]

            else:
                if direction == Direction.LEFT:
                    direction_int = 1
                    # 当前路
                    continuous_laneid["Middle"].append(current_lane)
                    continuous_length[current_lane] = TrafficInterface.myGetLaneLength(current_lane)
                    continuous_shape[current_lane] = Dao.dicLaneShapes[current_lane]
                    # 下一条路
                    next_lane = Dao.lane_dictionary[current_lane].next_lane[0]
                    continuous_laneid["Middle"].append(next_lane)
                    continuous_length[next_lane] = TrafficInterface.myGetLaneLength(next_lane)
                    continuous_shape[next_lane] = Dao.dicLaneShapes[next_lane]
                    # 下下一条路
                    next_next_lane = Dao.lane_dictionary[next_lane].next_lane[0]
                    continuous_laneid["Middle"].append(next_next_lane)
                    continuous_length[next_next_lane] = TrafficInterface.myGetLaneLength(next_next_lane)
                    continuous_shape[next_next_lane] = Dao.dicLaneShapes[next_next_lane]
                    # 左边的路
                    left_lane = SimPlatformAPI.myGetLeftLaneID(vehicle.id)
                    continuous_laneid["Left"].append(left_lane)
                    continuous_length[left_lane] = TrafficInterface.myGetLaneLength(left_lane)
                    continuous_shape[left_lane] = Dao.dicLaneShapes[left_lane]
                    # 左边的路的下一条路
                    try:
                        left_next_lane = Dao.lane_dictionary[left_lane].next_lane[0]
                    except:
                        log.info('left_lane has no next lane')
                    else:
                        continuous_laneid["Left"].append(left_next_lane)
                        continuous_length[left_next_lane] = TrafficInterface.myGetLaneLength(left_next_lane)
                        continuous_shape[left_next_lane] = Dao.dicLaneShapes[left_next_lane]
                        # 左边的路的下一条路的下一条路
                        try:
                            left_next_next_lane = Dao.lane_dictionary[left_next_lane].next_lane[0]
                        except:
                            log.info('next_lane of left_lane has no next lane')
                        else:
                            continuous_laneid["Left"].append(left_next_next_lane)
                            continuous_length[left_next_next_lane] = TrafficInterface.myGetLaneLength(
                                left_next_next_lane)
                            continuous_shape[left_next_next_lane] = Dao.dicLaneShapes[left_next_next_lane]

                elif direction == Direction.RIGHT:
                    direction_int = 2
                    # 当前路
                    continuous_laneid["Middle"].append(current_lane)
                    continuous_length[current_lane] = TrafficInterface.myGetLaneLength(current_lane)
                    continuous_shape[current_lane] = Dao.dicLaneShapes[current_lane]
                    # 下一条路
                    next_lane = Dao.lane_dictionary[current_lane].next_lane[0]
                    continuous_laneid["Middle"].append(next_lane)
                    continuous_length[next_lane] = TrafficInterface.myGetLaneLength(next_lane)
                    continuous_shape[next_lane] = Dao.dicLaneShapes[next_lane]
                    # 下下一条路
                    next_next_lane = Dao.lane_dictionary[next_lane].next_lane[0]
                    continuous_laneid["Middle"].append(next_next_lane)
                    continuous_length[next_next_lane] = TrafficInterface.myGetLaneLength(next_next_lane)
                    continuous_shape[next_next_lane] = Dao.dicLaneShapes[next_next_lane]
                    # 右边的路
                    right_lane = SimPlatformAPI.myGetRightLaneID(vehicle.id)
                    continuous_laneid["Right"].append(right_lane)
                    continuous_length[right_lane] = TrafficInterface.myGetLaneLength(right_lane)
                    continuous_shape[right_lane] = Dao.dicLaneShapes[right_lane]
                    # 右边的路的下一条路
                    try:

                        right_next_lane = Dao.lane_dictionary[right_lane].next_lane[0]
                    except:
                        log.info('right_lane has no next lane')
                    else:
                        continuous_laneid["Right"].append(right_next_lane)
                        continuous_length[right_next_lane] = TrafficInterface.myGetLaneLength(right_next_lane)
                        continuous_shape[right_next_lane] = Dao.dicLaneShapes[right_next_lane]

                        # 右边边的路的下一条路的下一条路
                        try:
                            right_next_next_lane = Dao.lane_dictionary[right_next_lane].next_lane[0]
                        except:
                            log.info('next_lane of right_lane has no next lane')
                        else:
                            continuous_laneid["Right"].append(right_next_next_lane)
                            continuous_length[right_next_next_lane] = TrafficInterface.myGetLaneLength(
                                right_next_next_lane)
                            continuous_shape[right_next_next_lane] = Dao.dicLaneShapes[right_next_next_lane]

        log.info("this car {} going to calculate track in multithread".format(vehicle.id))

        # vehicle_length = SimPlatformAPI.myGetVehicleLength(vehicle.id)
        MultiThreadPool.calculate_way_point(trajectory,
                                            s_from_lane_start,
                                            lane_width,
                                            distance,
                                            direction_int,
                                            continuous_laneid,
                                            continuous_length,
                                            continuous_shape)
        return

    # @staticmethod
    # def move_to(vehID, x, y, yaw):
    #     curLaneID = myGetLaneID(vehID)
    #     if curLaneID == "":
    #         return
    #     if SimPlatformAPI.myIsDeadEnd(curLaneID):
    #         station = myGetDistanceFromLaneStart(vehID)
    #         # todo 下一行的条件考虑换用PanoSimTrafficAPI2.myGetDistanceToLaneEnd(vehID)<5
    #         if TrafficInterface.get_lane_length(curLaneID) - station < 5:
    #             return
    #     degYaw = yaw * 180 / math.pi
    #     SimPlatformAPI.my_Move_To(vehID, x, y, degYaw)

    @classmethod
    def move_to(cls,vehID, x, y, degYaw,lane,s,l):
        degYaw = degYaw * 180 / math.pi
        SimPlatformAPI.myMoveTo(vehID, x, y, degYaw, lane, s, l)

        #SimPlatformAPI.my_Move_To(vehID, x, y, degYaw)
