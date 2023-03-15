# from abc import ABCMeta, abstractmethod  # 实现接口的模块
from Library.package_platformAPI.ExternalInterface import ExternalInterface

class ExternalInterfaceImpl(ExternalInterface):
    e=None

    @classmethod
    def initial(cls,sumo_file_name):
        cls.lane_manager.initial(sumo_file_name)
        cls.vehicle_manager.initial()


    @classmethod
    def getJunctionList(cls):
        return cls.lane_manager.junctionList

    @classmethod
    def getIncomingLanes(cls, junctionId):
        result=cls.lane_manager.get_incoming_lanes(junctionId)
        return result

    @classmethod
    def getLaneShape(cls,laneID):
        result=cls.lane_manager.get_lane_shape(laneID)
        return result

    @classmethod
    def getInternalLanes(cls, junctionId):
        result = cls.lane_manager.get_internal_lanes(junctionId)
        return result

    @classmethod
    def getValidDirections(cls, laneID):
        result = cls.lane_manager.get_valid_direction(laneID)
        return result #返回list

    @classmethod
    def isInternalLane(cls,laneID):
        result = cls.lane_manager.is_internal_lane(laneID)
        return result

    @classmethod
    def getVehicleLengthById(cls,vehicle_id):
        result = cls.vehicle_manager.get_vehicle_length_by_id(vehicle_id)
        return result

    @classmethod
    def getVehicleSpeed(cls,vehicle_id):
        result =cls.vehicle_manager.get_speed(vehicle_id)
        return result

    @classmethod
    def getVehicleX(cls,vehID):
        return cls.vehicle_manager.get_vehicle_x(vehID)

    @classmethod
    def getVehicleY(cls,vehID):
        return cls.vehicle_manager.get_vehicle_y(vehID)

    @classmethod
    def getVehicleLane(cls,vehID):
        return cls.vehicle_manager.get_vehicle_lane(vehID)

    @classmethod
    def getDistanceToLaneEnd(cls,vehID):
        distance_from_lane_start= cls.getDistanceFromLaneStart(vehID)
        current_lane_id=cls.vehicle_manager.get_vehicle_lane(vehID)
        lane_length = float(cls.lane_manager.get_lane_length(current_lane_id))
        return lane_length-distance_from_lane_start

    @classmethod
    def getDistanceFromLaneStart(cls,vehID):
        return cls.vehicle_manager.get_distance_from_lane_start(vehID)

    @classmethod #TODO 这里应该是围绕当前车画一个矩形或圆，看左边车道的车是不是在这个范围内，这样会准确一点
    def getLeftLeaderVehicle(cls,vehID):
        '''

        :param vehID: string
        :return: string
        '''
        distance_from_lane_start=cls.getDistanceFromLaneStart(vehID)
        current_lane_id=cls.vehicle_manager.get_vehicle_lane(vehID)
        left_lane=cls.lane_manager.get_left_lane(current_lane_id)
        if left_lane is None:
            return None
        vehicles=cls.lane_manager.vehicles_on_lane[left_lane]

        target_vehicle=None
        temp_distance=50
        for each_vehicle in vehicles:
            each_distance_from_lane_start=cls.getDistanceFromLaneStart(each_vehicle)
            s_distance_from_me_to_each=each_distance_from_lane_start-distance_from_lane_start
            if s_distance_from_me_to_each>0 and s_distance_from_me_to_each<temp_distance:
                target_vehicle=each_vehicle
                temp_distance=s_distance_from_me_to_each
        return target_vehicle

    @classmethod
    def getLeftFollowerVehicle(cls,vehID):
        """

        :param vehID: string
        :return: string
        """
        distance_from_lane_start = cls.getDistanceFromLaneStart(vehID)
        current_lane_id = cls.vehicle_manager.get_vehicle_lane(vehID)
        left_lane = cls.lane_manager.get_left_lane(current_lane_id)
        if left_lane is None:
            return None
        vehicles = cls.lane_manager.vehicles_on_lane[left_lane]

        target_vehicle = None
        temp_distance = 50
        for each_vehicle in vehicles:
            each_distance_from_lane_start = cls.getDistanceFromLaneStart(each_vehicle)
            s_distance_from_each_to_me = distance_from_lane_start - each_distance_from_lane_start
            if s_distance_from_each_to_me > 0 and s_distance_from_each_to_me < temp_distance:
                target_vehicle = each_vehicle
                temp_distance = s_distance_from_each_to_me
        return target_vehicle

    @classmethod
    def getRightLeaderVehicle(cls,vehID):
        '''

        :param vehID: string
        :return: string
        '''
        distance_from_lane_start = cls.getDistanceFromLaneStart(vehID)
        current_lane_id = cls.vehicle_manager.get_vehicle_lane(vehID)
        right_lane = cls.lane_manager.get_right_lane(current_lane_id)
        if right_lane is None:
            return None
        vehicles = cls.lane_manager.vehicles_on_lane[right_lane]

        target_vehicle = None
        temp_distance = 50
        for each_vehicle in vehicles:
            each_distance_from_lane_start = cls.getDistanceFromLaneStart(each_vehicle)
            s_distance_from_me_to_each = each_distance_from_lane_start - distance_from_lane_start
            if s_distance_from_me_to_each > 0 and s_distance_from_me_to_each < temp_distance:
                target_vehicle = each_vehicle
                temp_distance = s_distance_from_me_to_each
        return target_vehicle

    @classmethod
    def getRightFollowerVehicle(cls,vehID):
        '''

        :param vehID: string
        :return: string
        '''
        distance_from_lane_start = cls.getDistanceFromLaneStart(vehID)
        current_lane_id = cls.vehicle_manager.get_vehicle_lane(vehID)
        right_lane = cls.lane_manager.get_right_lane(current_lane_id)
        if right_lane is None:
            return None
        vehicles = cls.lane_manager.vehicles_on_lane[right_lane]

        target_vehicle = None
        temp_distance = 50
        for each_vehicle in vehicles:
            each_distance_from_lane_start = cls.getDistanceFromLaneStart(each_vehicle)
            s_distance_from_each_to_me = distance_from_lane_start-each_distance_from_lane_start
            if s_distance_from_each_to_me > 0 and s_distance_from_each_to_me < temp_distance:
                target_vehicle = each_vehicle
                temp_distance = s_distance_from_each_to_me
        return target_vehicle

    @classmethod
    def getLeftLane(cls,laneID):
        return cls.lane_manager.get_left_lane(laneID)

    @classmethod
    def getRightLane(cls,laneID):
        return cls.lane_manager.get_right_lane(laneID)

    @classmethod
    def getVehicleList(cls):
        return cls.vehicle_manager.get_vehicle_list()

    @classmethod
    def getVehicleYaw(cls,vehID): #在上层经过了转化
        return cls.vehicle_manager.get_vehicle_yaw(vehID)

    @classmethod
    def isDeadEnd(cls,laneID):
        return cls.lane_manager.is_dead_end(laneID)

    @classmethod
    def getRoute(cls,vehID):
        return cls.vehicle_manager.get_route(vehID)

    @classmethod
    def getVehicleAccel(cls,vehID):
        return cls.vehicle_manager.get_vehicle_accel(vehID)

    @classmethod
    def getLeaderVehicle(cls,vehID):
        '''

        :param vehID: string
        :return: string
        '''
        distance_from_lane_start = cls.getDistanceFromLaneStart(vehID)
        current_lane_id = cls.vehicle_manager.get_vehicle_lane(vehID)
        vehicles = cls.lane_manager.vehicles_on_lane[current_lane_id]

        target_vehicle = None
        temp_distance = 50
        for each_vehicle in vehicles:
            if each_vehicle==vehID:
                continue
            each_distance_from_lane_start = cls.getDistanceFromLaneStart(each_vehicle)
            s_distance_from_me_to_each = each_distance_from_lane_start - distance_from_lane_start
            if s_distance_from_me_to_each > 0 and s_distance_from_me_to_each < temp_distance:
                target_vehicle = each_vehicle
                temp_distance = s_distance_from_me_to_each

        return target_vehicle

    @classmethod
    def changeSpeed(cls,car_id, current_speed):
        cls.vehicle_manager.change_speed(car_id, current_speed)

    @classmethod
    def changeRoute(cls,vehID,route):
        cls.vehicle_manager.change_route(vehID,route)

    @classmethod
    def getVehicleMaxSpeed(cls,vehID):
        return cls.vehicle_manager.get_vehicle_max_speed(vehID)

    @classmethod
    def getLaneVehicles(cls,laneID):
        return cls.lane_manager.get_lane_vehicles(laneID)

    @classmethod
    def getVehicleLength(cls,vehID):
        return cls.vehicle_manager.get_vehicle_length(vehID)

    @classmethod
    def getLaneLength(cls,laneID):
        return float(cls.lane_manager.get_lane_length(laneID))#string要转化为float

    @classmethod
    def getVehicleMaxAccel(cls,vehID):
        return cls.vehicle_manager.get_vehicle_max_acc(vehID)

    @classmethod
    def getVehicleMaxDecel(cls,vehID):
        return cls.vehicle_manager.get_vehicle_max_dec(vehID)

    @classmethod
    def getVehicleLateralOffset(cls,vehID):
        return cls.vehicle_manager.get_vehicle_lateral_offset(vehID)

    @classmethod
    def getVehicleType(cls,vehID):
        return cls.vehicle_manager.get_vehicle_type(vehID)

    @classmethod
    def getInternalFoeLanes(cls,lane_id):
        return cls.lane_manager.get_internal_foe_lanes(lane_id)

    @classmethod
    def moveTo(cls,vehID,x, y, yaw,lane,s,l):
        previous_lane=cls.vehicle_manager.get_vehicle_lane(vehID)
        if previous_lane!=lane:
            cls.lane_manager.delete_vehicle_on_lane(previous_lane,vehID)
            cls.lane_manager.add_vehicle_on_lane(lane,vehID)
        cls.vehicle_manager.move_to(vehID,x, y, yaw,lane,s,l)

    @classmethod
    def addVehicle(cls,x, y, current_s,lane_id, speed ,type):
        id=cls.vehicle_manager.addVehicle(x, y, current_s,lane_id, speed ,type)
        cls.lane_manager.add_vehicle_on_lane(lane_id, id)

    @classmethod
    def deleteVehicle(cls,vehID):
        cls.lane_manager.delete_vehicle_on_lane(cls.vehicle_manager.get_vehicle_lane(vehID), vehID)
        cls.vehicle_manager.delete_vehicle(vehID)


    @classmethod #TODO 记得在ENUM中把dir改成字符串
    def getAllNextLanes(cls,laneID, dir):
        return cls.lane_manager.get_all_next_lanes(laneID, dir)
