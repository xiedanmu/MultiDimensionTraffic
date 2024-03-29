import copy
import math
# import random
import numpy as np

from Library.package_entity.Enum_Driving_Mode import DrivingMode

rng = np.random.Generator(np.random.MT19937(1))

from Library.package_platformAPI.SimPlatformAPI import SimPlatformAPI
from Library.package_entity.Class_Trajectory import Trajectory
from Library.package_entity.Enum_Direction import Direction
from Library.package_entity.Enum_Located_Area import LocatedArea
from Library.package_entity.Enum_Types_Of_Road import TypesOfRoad
from Library.package_interface.Traffic_Interface import TrafficInterface
from Library.package_dao.Class_Dao import Dao
import logging

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())

class TrafficInterfaceImpl(TrafficInterface):

    """
    计算交通层面的接口
    TrafficInterface的层级比MotivateInterface低，即不能在这个文件中导入 MotivateInterface
    """

    def __init__(self):
        pass

    @staticmethod  # 声明为抽象方法，子类必须重写的方法
    def set_car_zone(vehicle):
        return

    @staticmethod
    def two_car_distance(car1:int, car2:int) -> float:
        """返回两车的直线距离"""
        lateral_distance = SimPlatformAPI.myGetVehicleX(car1) - SimPlatformAPI.myGetVehicleX(car2)

        # lateral_distance = SimPlatformAPI.myGetLateralDistance(car1_id, car2_id) 返回了负无穷
        longitudinal_distance= SimPlatformAPI.myGetVehicleY(car1) - SimPlatformAPI.myGetVehicleY(car2)
        return pow(longitudinal_distance ** 2 + lateral_distance ** 2, 1 / 2)

    # 函数: getXYFromSL
    # 用途: 根据S,L坐标获取目标车道的X,Y坐标
    # [i]: laneID
    # [i]: station 期望的S坐标
    # [i]: offset 期望的L坐标
    # [o]: (x,y) 期望的X,Y坐标
    # [example]: (x,y) = getXYFromSL( getLaneShape(id, -1/0/1), 100, 0 )
    # [o]: s<0 & s>S_max 会返回none值 造成类型报错（使用约束）
    @classmethod
    def getXYFromSL(cls, laneID, station, offset):
        points = cls.myGetCompleteLaneShape(laneID)
        if (len(points) <= 1):
            log.error("Error: No points in laneID {}".format(laneID))
            return (None, None)

        def calculateRatio(points, station):
            ratio = 0
            idx = i = 0

            if station > points[len(points) - 1][2]:
                if (station - points[len(points) - 1][2]) < 3:  # 误差在2（后面改为了3）以内都能接受 modified by xdm on 2021/12/28
                    idx = len(points) - 1
                    ratio = 1
                else:
                    idx = None
                    ratio = None

            elif station < 0:
                idx = None
                ratio = None

            elif station == points[len(points) - 1][2]:
                idx = len(points) - 1
                ratio = 1

            else:
                while i < len(points) - 1:
                    if (station >= points[i][2] and station < points[i + 1][2]):
                        idx = i
                        break
                    else:
                        i = i + 1

            if idx != None and idx != len(points) - 1:
                ratio = (station - points[idx][2]) / (points[idx + 1][2] - points[idx][2])
            return (idx, ratio)

        def linearInterpolation(x1, y1, x2, y2, ratio):
            if ratio == None:
                return (None, None)
            else:
                x = x1 + ratio * (x2 - x1)
                y = y1 + ratio * (y2 - y1)
                return (x, y)

        def pointOffset(xSrc, ySrc, yaw, offset):
            if xSrc == None or ySrc == None:
                return (None, None)
            else:
                x = None
                y = None
                if yaw <= math.pi / 2:  # yaw属于SUMO里面的第一象限
                    alaph = math.pi / 2 - yaw
                    x = xSrc - offset * math.sin(alaph)
                    y = ySrc + offset * math.cos(alaph)
                elif yaw <= math.pi:  # yaw属于SUMO里面的第二象限
                    alaph = yaw - math.pi / 2
                    x = xSrc + offset * math.cos(alaph)
                    y = ySrc + offset * math.sin(alaph)
                elif yaw <= math.pi * 3 / 2:  # yaw属于SUMO里面的第三象限
                    alaph = math.pi * 3 / 2 - yaw
                    x = xSrc + offset * math.sin(alaph)
                    y = ySrc - offset * math.cos(alaph)
                elif yaw <= math.pi * 2:  # yaw属于SUMO里面的第四象限
                    alaph = yaw - math.pi * 3 / 2
                    x = xSrc + offset * math.sin(alaph)
                    y = ySrc - offset * math.cos(alaph)
                else:
                    log.error("check why yaw >360")

                return (x, y)

        x = 0
        y = 0

        pointsWithS = points
        (idx, ratio) = calculateRatio(pointsWithS, station)
        if (idx == None or ratio == None):
            return (None, None)

        if idx == len(pointsWithS) - 1:
            (x, y) = pointOffset(pointsWithS[idx][0], pointsWithS[idx][1], pointsWithS[idx][3], offset)
            return (x, y)

        else:
            (xTmp, yTmp) = linearInterpolation(pointsWithS[idx][0], pointsWithS[idx][1], pointsWithS[idx + 1][0],
                                               pointsWithS[idx + 1][1], ratio)
            (x, y) = pointOffset(xTmp, yTmp, pointsWithS[idx][3], offset)
            # (x, y) = pointOffset(xTmp, yTmp, pointsWithS[idx][2], offset)
            return (x, y)

    # 函数: myGetCompleteLaneShape
    # 用途: 获得车道点集
    # [i]: laneID - 车道ID
    # [o]: [[x,y,s,yaw], ..., [x,y,s,yaw]] - 点集，带s和yaw
    @staticmethod
    def myGetCompleteLaneShape(laneID):
        if laneID in Dao.dicLaneShapes:
            return Dao.dicLaneShapes[laneID]
        else:
            return []

    # 函数: myGetLaneLength
    # 用途: 获得车道长度
    # [i]: laneID - 车道ID
    # [o]: length - 车道长度。 当未找到车道时，返回-1073741824
    @classmethod
    def myGetLaneLength(cls, laneID):
        shape = cls.myGetCompleteLaneShape(laneID)
        if len(shape) <= 0: return -1073741824
        return shape[len(shape) - 1][2]

    # 函数: check_lane_exist
    # 用途: 判断车辆是否存在左车道或右车道
    # [i]: vehID - 车辆ID
    # [i]: dir -   Direction.LEFT：左侧；  Direction.RIGHT右侧；
    # [o]: Bool - True有车道，False无车道
    @staticmethod
    def check_lane_exist(vehID, dir):
        if (vehID < 0):
            log.warning("the vehID is blow 0 in check_lane_exist")
            return False

        if dir == Direction.LEFT:  # 左侧
            tmpID = SimPlatformAPI.myGetLeftLeaderVehicle(vehID)
            log.info("GetLeftLeader {}".format(tmpID))
            if (tmpID >= 0) or (tmpID == -1):
                return True
            else:
                return False
        elif Direction.RIGHT:  # 右侧
            tmpID = SimPlatformAPI.myGetRightLeaderVehicle(vehID)
            log.info("GetRightLeader {}".format(tmpID))
            if (tmpID >= 0) or (tmpID == -1):
                return True
            else:
                return False
        else:
            log.info("Direction is invalid in check_lane_exist")

    # 函数: check_free_change_lane
    # 用途: 判断车辆向左自由换道和向右自由换道是否满足条件
    # [i]: vehID - 车辆ID
    #    计算过程中ABCD四个条件都能满足才行
    # [o]: Direction.LEFT:左侧;Direction.RIGHT:右侧;None
    @classmethod
    def get_free_change_lane_direction(cls, vehID):
        left_check_a = False
        left_check_b = False
        right_check_a = False
        right_check_b = False
        left_check = False
        right_check = False

        left_leader_distance = 0
        right_leader_distance = 0

        #路不够长了
        current_lane_id= SimPlatformAPI.myGetVehicleLane(vehID)
        if(SimPlatformAPI.myGetLaneLength(current_lane_id) - SimPlatformAPI.myGetDistanceFromLaneStart(vehID))<50:
            return None
        #todo 检查车道是否存在
        # 先看左侧换道
        left_lane=SimPlatformAPI.myGetLeftLaneID(vehID)
        SimPlatformAPI.isLaneExist(left_lane)
        if SimPlatformAPI.isLaneExist(left_lane):
            log.info("vehID {}, left lane {}".format(vehID, SimPlatformAPI.myGetLeftLaneID(vehID)))
            # 条件A开始检查
            left_leader_car = SimPlatformAPI.myGetLeftLeaderVehicle(vehID)
            if left_leader_car <= -1:  # 没有车就直接满足条件a
                left_check_a = True
            elif left_leader_car >= 0:
                left_leader_distance = cls.two_car_distance(Dao.vehicle_dictionary[vehID].id, Dao.vehicle_dictionary[left_leader_car].id)
                left_leader_v = Dao.vehicle_dictionary[left_leader_car].current_speed

                leader_car = SimPlatformAPI.myGetLeaderVehicle(vehID)
                if leader_car == -1:
                    if left_leader_distance > 20:
                        left_check_a = True
                elif leader_car >= 0:
                    leader_distance = cls.two_car_distance(Dao.vehicle_dictionary[vehID].id, Dao.vehicle_dictionary[leader_car].id)
                    leader_v = Dao.vehicle_dictionary[leader_car].current_speed
                    if left_leader_distance > leader_distance and left_leader_v > leader_v:
                        left_check_a = True
            # 条件A检查完毕

            # 条件B开始检查
            left_follow_car = SimPlatformAPI.myGetLeftFollowerVehicle(vehID)
            if left_follow_car <= -1:
                left_check_b = True

            elif left_follow_car >= 0:
                required_distance_min = Dao.vehicle_dictionary[left_follow_car].current_speed * 2
                current_distance = cls.two_car_distance(Dao.vehicle_dictionary[vehID].id, Dao.vehicle_dictionary[left_follow_car].id)
                if current_distance >= required_distance_min:
                    left_check_b = True
            # 条件B检查完毕

            # 左侧换道条件ABC的和运算
            if left_check_a and left_check_b:
                left_check = True

        # 再看右侧换道
        right_lane = SimPlatformAPI.myGetRightLaneID(vehID)
        SimPlatformAPI.isLaneExist(right_lane)
        if SimPlatformAPI.isLaneExist(right_lane):
            log.info("vehID {}, right lane {}".format(vehID, SimPlatformAPI.myGetRightLaneID(vehID)))
            # 条件A开始检查
            right_leader_car = SimPlatformAPI.myGetRightLeaderVehicle(vehID)
            if right_leader_car <= -1:  # 没有车就直接满足条件a
                right_check_a = True
            elif right_leader_car >= 0:
                right_leader_distance = cls.two_car_distance(Dao.vehicle_dictionary[vehID].id, Dao.vehicle_dictionary[right_leader_car].id)
                right_leader_v = Dao.vehicle_dictionary[right_leader_car].current_speed

                leader_car = SimPlatformAPI.myGetLeaderVehicle(vehID)
                if leader_car == -1:
                    if right_leader_distance > 20:
                        right_check_a = True
                elif leader_car >= 0:
                    leader_distance = cls.two_car_distance(Dao.vehicle_dictionary[vehID].id, Dao.vehicle_dictionary[leader_car].id)
                    leader_v = Dao.vehicle_dictionary[leader_car].current_speed
                    if right_leader_distance > leader_distance and right_leader_v > leader_v:
                        right_check_a = True
            # 条件A检查完毕

            # 条件B开始检查
            right_follow_car = SimPlatformAPI.myGetRightFollowerVehicle(vehID)
            if right_follow_car <= -1:
                right_check_b = True

            elif right_follow_car >= 0:
                required_distance_min = Dao.vehicle_dictionary[right_follow_car].current_speed * 2
                current_distance = cls.two_car_distance(Dao.vehicle_dictionary[vehID].id, Dao.vehicle_dictionary[right_follow_car].id)
                if current_distance >= required_distance_min:
                    right_check_b = True
            # 条件B检查完毕

            # 右侧侧换道条件ABC的和运算
            if right_check_a and right_check_b:
                right_check = True

        #  根据实际情况返回换道方向
        if not right_check and not left_check:
            return None
        elif right_check and left_check:
            random_value = rng.integers(0, 9)

            if random_value<5:
            # if left_leader_distance >= right_leader_distance:
                return Direction.LEFT
            else:
                return Direction.RIGHT
        elif left_check:
            return Direction.LEFT
        elif right_check:
            return Direction.RIGHT

    # 函数: get_lane_length
    # 用途: 获取道路长度
    # [i]: vehID - 车辆ID
    # [o]:
    @classmethod  # 声明为抽象方法，子类必须重写的方法
    def get_lane_length(cls, lane_id):
        shape = cls.myGetCompleteLaneShape(lane_id)
        if (len(shape) <= 0): return -1073741824
        return shape[len(shape) - 1][2]

    # 函数:  get_conflict_vehid_in_foelane_by_vehid
    # 用途: 获取冲突车道的所有车辆
    # [i]: int - vehID - 车辆ID
    # [o]: List - foe_veh_list 冲突车道上的所有车
    @classmethod  # 声明为抽象方法，子类必须重写的方法
    def get_conflict_vehid_in_foelane_by_vehid(cls, vehID):
        # todo 还未完成
        # list =SimPlatformAPI.getLaneVehicles(lane)
        return

    # 函数:  update_the_vehicle_area
    # 用途: 更新车辆的所在区域
    # [i]: current_lane - Lane ,Class_Vehicle - veh - 车辆对象实例
    # [o]: Bool - True Or False 是否成功
    @classmethod  # 声明为抽象方法，子类必须重写的方法
    def update_the_vehicle_area(cls,current_lane, vehicle):

        if current_lane.type == TypesOfRoad.INTERNAL_LANE:
            vehicle.located_area = LocatedArea.INTERNAL
            vehicle.current_lane = current_lane
        elif SimPlatformAPI.myGetDistanceToLaneEnd(vehicle.id) < 40:
            vehicle.located_area = LocatedArea.ADJACENT_JUNCTION_AREA
            vehicle.current_lane = current_lane
            # if vehicle.driving_mode==DrivingMode.LEFT_CHANGING_LANE or vehicle.driving_mode==DrivingMode.RIGHT_CHANGING_LANE:
            #    MotivateInterface.generate_trajectory_point_set(vehicle)
        else:
            vehicle.located_area = LocatedArea.NORMAL
            vehicle.current_lane = current_lane

        return True

    # 函数: get_direction_from_lane_to_lane
    # 用途: 获取lane1到lane2是什么方向
    # [i]: int,int - lane1id,lane2id - 道路的id
    # [o]: int - direction 左，直行，右
    @classmethod  # 声明为抽象方法，子类必须重写的方法
    def get_direction_from_lane_to_lane(cls,current_lane_id, next_lane_id):

        result_direction=None
        dirs = SimPlatformAPI.myGetValidDirections(current_lane_id)
        if (len(dirs) <= 0):
            log.warning("[{}] has no directions".format(current_lane_id))
        for dir in dirs:
            toLanes = SimPlatformAPI.myGetNextLanes(current_lane_id, dir)

            if next_lane_id in toLanes:
                result_direction=dir

        return result_direction

    # 函数: get_leader_vehicle
    # 用途: 获取leader vehicle
    # [i]: int - carID 车辆ID
    # [o]: int - leader_carID
    @classmethod  # 声明为抽象方法，子类必须重写的方法
    def get_leader_vehicle(cls, vehicle_id,trajectory):
        leader_id = SimPlatformAPI.myGetLeaderVehicle(vehicle_id)

        vehicle=Dao.get_vehicle_by_id(vehicle_id)
        try:#有主车的情况下，需要try catch
            if vehicle.driving_mode == DrivingMode.FOLLOW_DRIVING:
                current_lane_id = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0][2]

                # 看一下刚迈出停止线时的情况
                if SimPlatformAPI.myCheckInternalLane(current_lane_id):
                    if leader_id <= -1:
                        current_lane = Dao.lane_dictionary[current_lane_id]
                        previous_lane_id = current_lane.previous_lane[0]
                        previous_lane = Dao.lane_dictionary[previous_lane_id]
                        my_current_s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle_id)
                        if previous_lane.type == TypesOfRoad.NORMAL_LANE and my_current_s < 20:
                            all_next_lanes = previous_lane.next_lane

                            target_vehicle = -1
                            temp = 100
                            for next_lane_each in all_next_lanes:
                                if next_lane_each == current_lane_id:
                                    continue
                                next_lane_vehicles = SimPlatformAPI.myGetLaneVehicles(next_lane_each)
                                for item in next_lane_vehicles:
                                    distance_from_lane_start = SimPlatformAPI.myGetDistanceFromLaneStart(item)
                                    if distance_from_lane_start < temp and distance_from_lane_start > my_current_s:
                                        temp = distance_from_lane_start
                                        target_vehicle = item

                            if target_vehicle > -1:
                                return target_vehicle

                        elif leader_id > -1:
                            current_lane = Dao.lane_dictionary[current_lane_id]
                            previous_lane_id = current_lane.previous_lane[0]
                            previous_lane = Dao.lane_dictionary[previous_lane_id]
                            my_current_s = SimPlatformAPI.myGetDistanceFromLaneStart(vehicle_id)
                            if previous_lane.type == TypesOfRoad.NORMAL_LANE and my_current_s < 20:
                                all_next_lanes = previous_lane.next_lane

                                target_vehicle = -1
                                temp = 100
                                for next_lane_each in all_next_lanes:
                                    if next_lane_each == current_lane_id:
                                        continue
                                    next_lane_vehicles = SimPlatformAPI.myGetLaneVehicles(next_lane_each)
                                    for item in next_lane_vehicles:
                                        distance_from_lane_start = SimPlatformAPI.myGetDistanceFromLaneStart(item)
                                        if distance_from_lane_start < temp and distance_from_lane_start > my_current_s and \
                                                distance_from_lane_start < SimPlatformAPI.myGetDistanceFromLaneStart(
                                            leader_id):
                                            temp = distance_from_lane_start
                                            target_vehicle = item

                                if target_vehicle > -1:
                                    return target_vehicle

                # 看一下在接近lane end处且有next_lane的情况
                if leader_id <= -1 and not SimPlatformAPI.myIsDeadEnd(current_lane_id) and Dao.lane_dictionary[
                    current_lane_id].next_lane and \
                        SimPlatformAPI.myGetDistanceToLaneEnd(vehicle_id) < 10:  # 前方没有车
                    all_next_lanes = cls.get_all_next_lane(trajectory)
                    target_vehicle = -1
                    temp = 100
                    next_next_lanes = []
                    for next_lane_each in all_next_lanes:
                        if SimPlatformAPI.myCheckInternalLane(next_lane_each):
                            next_next_lane = Dao.lane_dictionary[next_lane_each].next_lane
                            if next_next_lane:
                                next_next_lanes.append(next_next_lane[0])

                        next_lane_vehicles = SimPlatformAPI.myGetLaneVehicles(next_lane_each)

                        for item in next_lane_vehicles:
                            distance_from_lane_start = SimPlatformAPI.myGetDistanceFromLaneStart(item)
                            if distance_from_lane_start < temp:
                                temp = distance_from_lane_start
                                target_vehicle = item

                    # 在internal lane中没有找到
                    if target_vehicle == -1:
                        for next_lane_each in next_next_lanes:
                            next_lane_vehicles = SimPlatformAPI.myGetLaneVehicles(next_lane_each)

                            for item in next_lane_vehicles:
                                distance_from_lane_start = SimPlatformAPI.myGetDistanceFromLaneStart(item)
                                if distance_from_lane_start < temp:
                                    temp = distance_from_lane_start
                                    target_vehicle = item

                    if target_vehicle > -1:
                        return target_vehicle

                # 不在离路口10米内，且不是死路，需要观察左右换道的车
                left_leader = SimPlatformAPI.myGetLeftLeaderVehicle(vehicle_id)
                right_leader = SimPlatformAPI.myGetRightLeaderVehicle(vehicle_id)

                if leader_id > -1:
                    temp_leader = leader_id
                    leader_distance = cls.two_car_distance(vehicle_id,
                                                           leader_id) - SimPlatformAPI.myGetVehicleLength(leader_id)
                    if left_leader > 0 and Dao.vehicle_dictionary[
                        left_leader].driving_mode == DrivingMode.RIGHT_CHANGING_LANE:
                        left_leader_distance = cls.two_car_distance(vehicle_id,
                                                                    left_leader) - SimPlatformAPI.myGetVehicleLength(
                            left_leader)
                        if left_leader_distance < leader_distance: #and SimPlatformAPI.myGetVehicleSpeed(
                                #vehicle_id) < SimPlatformAPI.myGetVehicleSpeed(left_leader):
                            leader_distance = left_leader_distance
                            temp_leader = left_leader

                    if right_leader > 0 and Dao.vehicle_dictionary[
                        right_leader].driving_mode == DrivingMode.LEFT_CHANGING_LANE:
                        right_leader_distance = cls.two_car_distance(vehicle_id,
                                                                     leader_id) - SimPlatformAPI.myGetVehicleLength(
                            right_leader)
                        if right_leader_distance < leader_distance:# and SimPlatformAPI.myGetVehicleSpeed(
                                #vehicle_id) < SimPlatformAPI.myGetVehicleSpeed(right_leader):
                            leader_distance = right_leader_distance
                            temp_leader = right_leader

                    # # 如果离得太近了，直行车道就不用避让变道的车
                    # if leader_distance < SimPlatformAPI.myGetVehicleLength(temp_leader) + 5:
                    #     return leader_id
                    # else:
                    #     return temp_leader
                    return temp_leader

                # 当前车道没有前车的情况下
                else:
                    temp_leader = leader_id
                    leader_distance = 100
                    if left_leader > 0 and Dao.vehicle_dictionary[
                        left_leader].driving_mode == DrivingMode.RIGHT_CHANGING_LANE:
                        left_leader_distance = cls.two_car_distance(vehicle_id,
                                                                    left_leader) - SimPlatformAPI.myGetVehicleLength(
                            left_leader)

                        if left_leader_distance < leader_distance and SimPlatformAPI.myGetVehicleSpeed(
                                vehicle_id) < SimPlatformAPI.myGetVehicleSpeed(left_leader):
                            leader_distance = left_leader_distance
                            temp_leader = left_leader

                    if right_leader > 0 and Dao.vehicle_dictionary[
                        right_leader].driving_mode == DrivingMode.LEFT_CHANGING_LANE:
                        right_leader_distance = cls.two_car_distance(vehicle_id,
                                                                     right_leader) - SimPlatformAPI.myGetVehicleLength(
                            right_leader)
                        if right_leader_distance < leader_distance and SimPlatformAPI.myGetVehicleSpeed(
                                vehicle_id) < SimPlatformAPI.myGetVehicleSpeed(right_leader):
                            leader_distance = right_leader_distance
                            temp_leader = right_leader

                    return temp_leader

            elif vehicle.driving_mode == DrivingMode.LEFT_CHANGING_LANE:

                left_leader = SimPlatformAPI.myGetLeftLeaderVehicle(vehicle_id)

                current_lane = SimPlatformAPI.myGetLaneID(vehicle.id)
                origin_lane = trajectory.continuous_lane_id[0]

                if current_lane != origin_lane:
                    return leader_id

                if left_leader > 0:
                    if leader_id <= -1:
                        return left_leader
                    else:
                        leader_distance = cls.two_car_distance(vehicle_id,
                                                               leader_id) - SimPlatformAPI.myGetVehicleLength(
                            leader_id)
                        left_leader_distance = cls.two_car_distance(vehicle_id,
                                                                    left_leader) - SimPlatformAPI.myGetVehicleLength(
                            left_leader)
                        if left_leader_distance < leader_distance:
                            return left_leader
                        else:
                            return leader_id
                else:
                    return leader_id

            elif vehicle.driving_mode == DrivingMode.RIGHT_CHANGING_LANE:

                right_leader = SimPlatformAPI.myGetRightLeaderVehicle(vehicle_id)

                current_lane = SimPlatformAPI.myGetLaneID(vehicle.id)
                origin_lane = trajectory.continuous_lane_id[0]

                if current_lane != origin_lane:
                    return leader_id

                if right_leader > 0:
                    if leader_id <= -1:
                        return right_leader
                    else:
                        leader_distance = cls.two_car_distance(vehicle_id,
                                                               leader_id) - SimPlatformAPI.myGetVehicleLength(
                            leader_id)
                        right_leader_distance = cls.two_car_distance(vehicle_id,
                                                                     right_leader) - SimPlatformAPI.myGetVehicleLength(
                            right_leader)
                        if right_leader_distance < leader_distance:
                            return right_leader
                        else:
                            return leader_id
                else:
                    return leader_id
        except:
            return -1


    # 函数: get_next_lane
    # 用途: 获取leader vehicle
    # [i]: int - carID 车辆ID
    # [o]: int - leader_carID
    @classmethod  # 声明为抽象方法，子类必须重写的方法
    def get_next_lane_by_trajectory(cls,trajectory:Trajectory):
        current_lane_id = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0][2]
        #没有加dead_lane的条件check
        cur_pos=trajectory.current_continuous_lane_id.index(current_lane_id)
        if cur_pos == len(trajectory.current_continuous_lane_id)-1:
            return Dao.lane_dictionary[current_lane_id].next_lane[0] #这里不是很严谨，trajectory中若没有next_lane了，这个nextlane可以选左转、右转或者直行
        next_lane=trajectory.current_continuous_lane_id[cur_pos+1]
        return next_lane

    @classmethod  # 声明为抽象方法，子类必须重写的方法
    def get_all_next_lane(cls,trajectory:Trajectory):
        current_lane_id = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[0][2]
        #没有加dead_lane的条件check
        return Dao.lane_dictionary[current_lane_id].next_lane #这里返回所有的nextlane，包括左转、右转或者直行


    target_lane_id_list_initially = [] #需要存内存的数据
    target_lane_id_list_per_step=[]
    total_lane_length = 0   #需要存内存的数据
    initial_done=0
    max_vehicle_num=99
    @classmethod
    def vehicle_generation(cls, generateMode=0, initial_density=0, arrivePossible=0):
        """
        用途: 在地图上生成汽车
        [i]: int - car_num 汽车数量
        [o]: None
        [参数解释]
        generateMode = 0, 0 - -开放性边界条件（消失的车和补充的无关），1 - -周期性边界条件（消失的车辆会产生重新从开头进入路网）
        initial Density = 0, 初始全路网（不含内部车道）的车辆密度，需要转化为车辆数目：（密度 * 道路长度 /（车辆长度 + 2）后取整，均匀分布到正常车道）
        arrivePossible = 0，对应于开放性边界条件下车辆进入路网的可能性，
        """

        def evaluate_total_length():
            """
            用途:计算全地图normal lane的总长度
            """
            if cls.total_lane_length==0:
                for lane in Dao.lane_dictionary.values():
                    if lane.type == TypesOfRoad.NORMAL_LANE:# and not SimPlatformAPI.myIsDeadEnd(lane.id): #禁止deadend
                        cls.total_lane_length += SimPlatformAPI.myGetLaneLength(lane.id)

            return cls.total_lane_length

            raise NotImplementedError

        def get_target_lanes_initially():
            """
            用途:计算全地图不是dead end的normal lane的条数
            """
            if cls.target_lane_id_list_initially==[]:
                for lane in Dao.lane_dictionary.values():
                    if lane.type == TypesOfRoad.NORMAL_LANE: #禁止deadend
                        cls.target_lane_id_list_initially.append(lane.id)
            return cls.target_lane_id_list_initially

        def get_target_lanes_each_step():
            """
            用途:计算全地图不是dead end的normal lane的条数
            """
            if cls.target_lane_id_list_per_step==[]:
                for lane in Dao.lane_dictionary.values():
                    if lane.type == TypesOfRoad.NORMAL_LANE and not SimPlatformAPI.myIsDeadEnd(lane.id) and len(Dao.lane_dictionary[lane.id].previous_lane)==0: #禁止deadend
                        cls.target_lane_id_list_per_step.append(lane.id)

                if cls.target_lane_id_list_per_step==[]:
                    for lane in Dao.lane_dictionary.values():
                        if lane.type == TypesOfRoad.NORMAL_LANE:  # 禁止deadend
                            cls.target_lane_id_list_per_step.append(lane.id)

            return cls.target_lane_id_list_per_step

        def convert_to_vehicles_initially(num,total_length):
            # x = 0
            # y = 0
            if num>=cls.max_vehicle_num:
                num=cls.max_vehicle_num
            current_s=rng.integers(1,4)
            target_lane_id_list_temp=get_target_lanes_initially()
            num_of_vehicle_now = len(Dao.vehicle_dictionary)
            if num > num_of_vehicle_now:
                index=0
                target_lane_id = target_lane_id_list_temp[index]
                for i in range(num - num_of_vehicle_now):
                    if current_s> SimPlatformAPI.myGetLaneLength(target_lane_id):
                        index += 1
                        current_s = rng.integers(2, 10)
                        if index >= len(target_lane_id_list_temp):
                            index = 0
                            current_s = 0

                    target_lane_id = target_lane_id_list_temp[index]
                    try:
                        (x, y) = cls.getXYFromSL(target_lane_id, current_s, 0)
                        # TODO speed需要设计
                        SimPlatformAPI.myAddVehicle(x=x, y=y, current_s=current_s,lane_id=target_lane_id,speed=0, type=rng.integers(1, 4))
                    except:
                        print('can\'t calculate x and y or something wrong with AddVehicle ')
                    else:
                        current_s+=total_length/rng.integers(num,num+10)
                        # if index >= len(target_lane_id_list_temp):
                        #     index=0
                        #     current_s = 0

        def convert_to_vehicles_per_step(num):
            current_s = 0
            target_lane_id_list_temp = get_target_lanes_each_step()
            num_of_vehicle_now = len(Dao.vehicle_dictionary)
            if num>=cls.max_vehicle_num:
                num=cls.max_vehicle_num
            if num > num_of_vehicle_now:
                index = rng.integers(0,len(target_lane_id_list_temp))
                target_lane_id = target_lane_id_list_temp[index]
                for i in range(num - num_of_vehicle_now):
                    if current_s > SimPlatformAPI.myGetLaneLength(target_lane_id):
                        index += 1
                        current_s=0
                    target_lane_id = target_lane_id_list_temp[index]
                    (x, y) = cls.getXYFromSL(target_lane_id, current_s, 0)
                    SimPlatformAPI.myAddVehicle(x=x, y=y, current_s=current_s,lane_id=target_lane_id,speed=0, type=rng.integers(1, 4))
                    index += 1
                    if index >= len(target_lane_id_list_temp):
                        index = 0
                        current_s += total_length/10/initial_density+rng.integers(0,3)

        def convert_to_vehicles_in_possibility():
            current_s=0
            target_lane_id_list=get_target_lanes_each_step()
            target_lane_id = rng.choice(target_lane_id_list)
            (x,y)=cls.getXYFromSL(target_lane_id,current_s,0)
            SimPlatformAPI.myAddVehicle(x=x, y=y, current_s=current_s,lane_id=target_lane_id,speed=0, type=rng.integers(1, 4))


        if generateMode==0: #开放性边界条件
            if cls.initial_done==0:
                total_length = evaluate_total_length()
                num = int(total_length/1000 * initial_density)
                convert_to_vehicles_initially(num,total_length)
                cls.initial_done = 1
            elif cls.initial_done==1:
                n=rng.random() #生成0到1之间的浮点数
                # if arrivePossible>1:
                #     print('arrivePossible 只能介于0和1之间')
                #     raise ValueError
                if n<arrivePossible/100: #一秒钟之内有arrivePossible这么大的可能性进车，10ms一个仿真步
                    convert_to_vehicles_in_possibility()
                pass
            return
        elif generateMode==1: #周期性边界条件
            if cls.initial_done == 0:
                total_length = evaluate_total_length()
                num=int(total_length/1000*initial_density)
                convert_to_vehicles_initially(num,total_length)
                cls.initial_done = 1
            elif cls.initial_done==1:
                total_length = evaluate_total_length()
                num = int(total_length / 1000 * initial_density)
                convert_to_vehicles_per_step(num)
            return
        else:
            print('generateMode 只能是1或者0')
            raise ValueError

        raise NotImplementedError

    cumulate_expect_generated_car=0.0
    @classmethod
    def vehicle_generation_flow(cls,flow,v_mean):
        '''
        函数: vehicle_generation_flow
        用途: 根据流量和平均车速在地图上生成汽车
        [i]: int - flow 流量 多少车辆/每小时
        [i]: int - v_mean 平均车速 地图上的平均车速 km/h
        [o]: None
        [还用到了的变量]
        target_lane_id_list_initially = [] #需要存内存的数据
        target_lane_id_list_per_step=[]
        total_lane_length = 0   #需要存内存的数据
        initial_done=0
        max_vehicle_num=99
        cumulate_expect_generated_car
        '''
        def evaluate_total_length():
            """
            用途:计算全地图normal lane的总长度
            """
            if cls.total_lane_length==0:
                for lane in Dao.lane_dictionary.values():
                    if lane.type == TypesOfRoad.NORMAL_LANE:# and not SimPlatformAPI.myIsDeadEnd(lane.id): #禁止deadend
                        cls.total_lane_length += SimPlatformAPI.myGetLaneLength(lane.id)

            return cls.total_lane_length

            raise NotImplementedError

        def get_target_lanes_initially():
            """
            用途:计算全地图不是dead end的normal lane的条数
            """
            if cls.target_lane_id_list_initially==[]:
                for lane in Dao.lane_dictionary.values():
                    if lane.type == TypesOfRoad.NORMAL_LANE: #禁止deadend
                        cls.target_lane_id_list_initially.append(lane.id)
            return cls.target_lane_id_list_initially

        def get_target_lanes_each_step():
            """
            用途:计算全地图不是dead end的normal lane的条数
            """
            if cls.target_lane_id_list_per_step==[]:
                for lane in Dao.lane_dictionary.values():
                    if lane.type == TypesOfRoad.NORMAL_LANE and not SimPlatformAPI.myIsDeadEnd(lane.id) and len(Dao.lane_dictionary[lane.id].previous_lane)==0: #禁止deadend
                        cls.target_lane_id_list_per_step.append(lane.id)

                if cls.target_lane_id_list_per_step==[]:
                    for lane in Dao.lane_dictionary.values():
                        if lane.type == TypesOfRoad.NORMAL_LANE:  # 禁止deadend
                            cls.target_lane_id_list_per_step.append(lane.id)

            return cls.target_lane_id_list_per_step

        def convert_to_vehicles_initially(num,total_length):
            # x = 0
            # y = 0
            if num>=cls.max_vehicle_num:
                num=cls.max_vehicle_num
            current_s=rng.integers(1,4)
            target_lane_id_list_temp=get_target_lanes_initially()
            num_of_vehicle_now = len(Dao.vehicle_dictionary)
            if num > num_of_vehicle_now:
                index=0
                target_lane_id = target_lane_id_list_temp[index]
                for i in range(num - num_of_vehicle_now):
                    if current_s> SimPlatformAPI.myGetLaneLength(target_lane_id):
                        index += 1
                        current_s = rng.integers(2, 10)
                        if index >= len(target_lane_id_list_temp):
                            index = 0
                            current_s = 0

                    target_lane_id = target_lane_id_list_temp[index]
                    try:
                        (x, y) = cls.getXYFromSL(target_lane_id, current_s, 0)
                        SimPlatformAPI.myAddVehicle(x=x, y=y, current_s=current_s,lane_id=target_lane_id,speed=0, type=rng.integers(1, 4))
                    except:
                        print('can\'t calculate x and y or something wrong with AddVehicle ')
                    else:
                        current_s+=total_length/rng.integers(num,num+10)
                        # if index >= len(target_lane_id_list_temp):
                        #     index=0
                        #     current_s = 0

        def convert_to_vehicles_per_step(num):
            current_s = 0
            target_lane_id_list_temp = get_target_lanes_each_step()

            index = rng.integers(0, len(target_lane_id_list_temp))
            target_lane_id = target_lane_id_list_temp[index]
            for i in range(num):
                if current_s > SimPlatformAPI.myGetLaneLength(target_lane_id):
                    index += 1
                    current_s = 0
                target_lane_id = target_lane_id_list_temp[index]
                (x, y) = cls.getXYFromSL(target_lane_id, current_s, 0)
                SimPlatformAPI.myAddVehicle(x=x, y=y, current_s=current_s,lane_id=target_lane_id,speed=0, type=rng.integers(1, 4))
                index += 1
                if index >= len(target_lane_id_list_temp):
                    index = 0
                    current_s += total_length / 10 / initial_density + rng.integers(0, 3)


        initial_density=flow/v_mean
        if cls.initial_done == 0:
            total_length = evaluate_total_length()
            num = int(total_length / 1000 * initial_density)
            convert_to_vehicles_initially(num, total_length)
            cls.initial_done = 1
        elif cls.initial_done == 1:
            num_of_vehicle_now = len(Dao.vehicle_dictionary)
            #if num_of_vehicle_now>flow/3600/100: #如果车太多了应该停发

            total_length = evaluate_total_length()
            cls.cumulate_expect_generated_car+=flow/3600/100 #10ms的量
            going_to_create_car_num=int(cls.cumulate_expect_generated_car)
            cls.cumulate_expect_generated_car-=going_to_create_car_num
            convert_to_vehicles_per_step(going_to_create_car_num)
            pass

        return

