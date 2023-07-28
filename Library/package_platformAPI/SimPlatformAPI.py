"""
这个文件是为了方便替换底层接口做的适配器
"""
import logging
import math

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())

class SimPlatformAPI:
    API_Name=None
    Behavior=None
    Helper=None # 只有PanoNewAPI需要原生Pano接口
    @classmethod
    def setTargetAPI(cls,APIname):
        if APIname == 'InteractionAPI':
            from Library.package_platformAPI.InteractionAPI import InteractionAPI
            cls.API_Name = APIname
            cls.Behavior = InteractionAPI
        elif APIname == 'PanoAPI':
            from Library.package_platformAPI.PanoSimTrafficAPI2 import PanoAPI
            cls.API_Name = APIname
            cls.Behavior = PanoAPI
        elif APIname == 'PanoNewAPI':
            import Library.PanoTrafficApi_Python备份
            from Library.package_platformAPI.PanoSimTrafficAPI2 import PanoAPI
            cls.API_Name = APIname
            cls.Helper=PanoAPI
            cls.Behavior = Library.PanoTrafficApi_Python备份
            cls.Behavior.PanoTrafficInterface.myInit("Traffic", "Traffic-A")
        elif APIname == 'SumoAPI':
            from Library.package_platformAPI.SumoAPI import SumoAPI
            cls.API_Name = APIname
            cls.Behavior = SumoAPI

    @classmethod
    def mySetObjectAgentControl(cls,objId: int, state: int, checkTerminate = 0, condition = 0.01):
        return cls.Behavior.mySetObjectAgentControl(objId,state, checkTerminate, condition)

    # @classmethod
    # def initLaneShapes(cls):
    #     cls.Behavior.initLaneShapes()

    @classmethod
    def myGetLaneToFoeLanes(cls,laneID): #此函数没有用到
        return cls.Behavior.myGetLaneToFoeLanes(laneID)

    # @classmethod
    # def myGetCompleteLaneShape(cls,laneID):
    #     return cls.Behavior.myGetCompleteLaneShape(laneID)

    @classmethod #这个函数没有使用到
    def myGetLaneLengthByLaneShape(cls,laneID):
        return cls.Behavior.myGetLaneLength(laneID)

    @classmethod
    def myGetVehicleLengthByTypeID(cls,typeId): #这个函数没有使用到
        return cls.Behavior.myGetVehicleLength(typeId)

    @classmethod
    def myGetVehicleWidthByType(cls, typeId):
        return cls.Behavior.myGetVehicleWidthByType(typeId)

    @classmethod
    def myGetVehicleWidth(cls, car_id):
        if cls.API_Name=='PanoNewAPI':
            return cls.Behavior.myGetVehicleWidth(car_id)
        else:
            return cls.Behavior.myGetVehicleWidthByType(cls.myGetVehicleType(car_id))

    # @classmethod
    # def getXYFromSL(cls,laneID, station, offset):
    #     return cls.Behavior.getXYFromSL(laneID, station, offset)

    @classmethod
    def myGetLaneShapeWithOnlyXY(cls, points): #这个函数没有使用到
        return cls.Behavior.myGetLaneShapeWithOnlyXY(points)

    @classmethod
    def myGetHostVehicleCoordinate(cls): #这个函数没有使用到
        return cls.Behavior.myGetHostVehicleCoordinate()

    @classmethod
    def myGetVehicleCoordinate(cls,vehID): #这个函数没有使用到
        return cls.Behavior.myGetVehicleCoordinate(vehID)

    @classmethod
    def myGetHostVehicleStationInLane(cls): #这个函数没有使用到
        return cls.Behavior.myGetHostVehicleStationInLane()

    @classmethod
    def myGetHostVehicleSpeed(cls): #这个函数没有使用到
        return cls.Behavior.myGetHostVehicleSpeed()

    @classmethod
    def myCheckHostVehicle(cls,vehID): #这个函数没有使用到
        return cls.Behavior.myCheckHostVehicle(vehID)

    @classmethod
    def myGetLeftLeaderVehicle(cls,vehID):
        leader= cls.Behavior.myGetLeftLeaderVehicle(vehID)
        if leader is None:
            return -1
        else:
            return int(leader)

    @classmethod
    def myGetRightLeaderVehicle(cls,vehID):
        leader = cls.Behavior.myGetRightLeaderVehicle(vehID)
        if leader is None:
            return -1
        else:
            return int(leader)

    @classmethod
    def myGetLeftFollowerVehicle(cls,vehID):
        Follower = cls.Behavior.myGetLeftFollowerVehicle(vehID)
        if Follower is None:
            return -1
        else:
            return int(Follower)

    @classmethod
    def myGetRightFollowerVehicle(cls,vehID):
        Follower = cls.Behavior.myGetRightFollowerVehicle(vehID)
        if Follower is None:
            return -1
        else:
            return int(Follower)

    @classmethod
    def myGetVehicleList(cls):
        return cls.Behavior.myGetVehicleList()

    @classmethod
    def myGetDistanceToLaneEnd(cls,vehID):
        return cls.Behavior.myGetDistanceToLaneEnd(vehID)

    @classmethod
    def myGetDistanceFromLaneStart(cls,vehID):
        return cls.Behavior.myGetDistanceFromLaneStart(vehID)

    @classmethod
    def myGetLaneID(cls,vehID): #这个函数没有使用到
        return cls.Behavior.myGetLaneID(vehID)

    @classmethod
    def myGetLeftLaneID(cls, vehID):
        if cls.API_Name=='PanoNewAPI':
            cur_lane_id=cls.Behavior.myGetVehicleLane(vehID)
            return cls.Behavior.myGetLeftLaneId(cur_lane_id)
        else:
            return cls.Behavior.myGetLeftLaneID(vehID)

    @classmethod
    def myGetRightLaneID(cls,vehID):
        if cls.API_Name=='PanoNewAPI':
            cur_lane_id=cls.Behavior.myGetVehicleLane(vehID)
            return cls.Behavior.myGetRightLaneId(cur_lane_id)
        else:
            return cls.Behavior.myGetRightLaneID(vehID)

    @classmethod
    def myGetLaneShape(cls,laneID):
        if cls.API_Name=='PanoNewAPI':
            return cls.Helper.myGetLaneShape(laneID)
        return list(cls.Behavior.myGetLaneShape(laneID))

    @classmethod
    def myCheckInternalLane(cls,laneID):
        if cls.API_Name=='PanoNewAPI':
            return cls.Behavior.myGetIsInternalLane(laneID)
        else:
            return cls.Behavior.myCheckInternalLane(laneID)

    @classmethod
    def myGetValidDirections(cls,laneID):
        if cls.API_Name == 'PanoNewAPI':
            return cls.Helper.myGetValidDirections(laneID)
        else:
            return cls.Behavior.myGetValidDirections(laneID)

    @classmethod
    def myGetNextLanes(cls,laneID, dir):
        return cls.Behavior.myGetNextLanes(laneID,dir)

    @classmethod
    def myGetAllNextLanes(cls,laneID, dir):
        if cls.API_Name=='PanoNewAPI':
            return cls.Helper.myGetAllNextLanes(laneID, dir)
        return cls.Behavior.myGetAllNextLanes(laneID, dir)

    @classmethod
    def myGetJunctionList(cls):

        junction_list = cls.Behavior.myGetJunctionList()
        return junction_list
        # temp_list=[]
        # for junction in junction_list:
        #     if junction.startswith(":"):
        #         continue
        #     temp_list.append(junction)
        # return temp_list

    @classmethod
    def myGetIncomingLanes(cls,junctionId):
        lanes = cls.Behavior.myGetIncomingLanes(junctionId)
        if lanes.__len__() == 0: # 对应interaction
            return []
        elif lanes[0] == '': # 对应SUMO
            return []
        return lanes

    @classmethod
    def myGetInternalLanes(cls,junctionId):
        return cls.Behavior.myGetInternalLanes(junctionId)

    @classmethod
    def myGetVehicleSpeed(cls,vehID):
        return cls.Behavior.myGetVehicleSpeed(vehID)

    @classmethod
    def myGetVehicleSpeedOrigin(cls,vehID):
        if cls.API_Name=='PanoNewAPI':
            return cls.Helper.myGetVehicleSpeed(vehID)
        else:
            raise NotImplementedError

    @classmethod
    def myGetVehicleX(cls,vehID):
        return cls.Behavior.myGetVehicleX(vehID)

    @classmethod
    def myGetVehicleY(cls,vehID):
        return cls.Behavior.myGetVehicleY(vehID)

    @classmethod
    def myGetVehicleYaw(cls,vehID):
        return cls.Behavior.myGetVehicleYaw(vehID)

    @classmethod
    def myTranslateYawFromSumoToNormal(cls,yaw): #这个函数没有使用到
        return -yaw + math.pi / 2

        # def myDeg2Rad(degree):
        #     return degree * math.pi / 180
        #
        # return myDeg2Rad(-yaw) + math.pi / 2  # 这里是从SUMO坐标系（正北，顺时针为正），转换为（正东，逆时针为正）

    @classmethod
    def myIsDeadEnd(cls,laneID):
        if cls.API_Name=='PanoNewAPI':
            return cls.Behavior.myGetIsDeadEnd(laneID)
        else:
            return cls.Behavior.myIsDeadEnd(laneID)

    @classmethod
    def myMoveTo(cls,vehID, x, y, yaw,lane,s,l,expSpeed):
        if cls.API_Name=='PanoNewAPI':
            cls.Behavior.myMoveObjectByXYYaw(vehID, x, y, yaw, expSpeed)
        else:
            cls.Behavior.myMoveTo(vehID, x, y, yaw,lane,s,l)

    @classmethod
    def myGetVehicleLane(cls,car_id):
        if cls.API_Name=='PanoNewAPI':
            return cls.Helper.myGetVehicleLane(car_id)
        else:
            return cls.Behavior.myGetVehicleLane(car_id)

    @classmethod
    def myGetVehicleAccel(cls,car_id):
        return cls.Behavior.myGetVehicleAccel(car_id)

    @classmethod
    def myGetLeaderVehicle(cls,car_id):
        if cls.API_Name=='PanoNewAPI':
            return cls.Behavior.myGetLeaderVehicleInLane(car_id)
        else:
            leader= cls.Behavior.myGetLeaderVehicle(car_id)
            if leader ==None:
                return -1
            else:
                return int(leader)
    
    @classmethod
    def myGetForwardLeaderVehicle(cls,car_id):
        if cls.API_Name=='PanoNewAPI':
            leader_info = cls.Behavior.myGetLeader(car_id)
            if leader_info[0]<=-1:
                return None
            else:
                return leader_info


    @classmethod
    def myChangeSpeed(cls,car_id, current_speed, duration):
        if cls.API_Name=='PanoNewAPI':
            cls.Behavior.myMoveVehicleBySpeed(car_id, current_speed, duration)
        else:
            cls.Behavior.myChangeSpeed(car_id,current_speed,duration)

    @classmethod
    def myGetVehicleRoute(cls,id):
        if cls.API_Name=='PanoNewAPI':
            return cls.Behavior.myGetRoute(id)
        return cls.Behavior.myGetVehicleRoute(id)

    @classmethod #Pano使用
    def myChangeRoute(cls,id, myLaneRute):
        cls.Behavior.myChangeRoute(id, myLaneRute)

    @classmethod
    def myGetVehicleMaxSpeed(cls,id):
        if cls.API_Name=='SumoAPI':
            return cls.Behavior.myGetVehicleMaxSpeed(id)*3.6
        elif cls.API_Name=='PanoNewAPI':
            return cls.Helper.myGetVehicleMaxSpeed(id)
        else:
            return cls.Behavior.myGetVehicleMaxSpeed(id)

    @classmethod
    def myGetInternalFoeLanes(cls,lane_id):
        if cls.API_Name=='PanoNewAPI':
            return cls.Helper.myGetInternalFoeLanes(lane_id)
        else:
            return cls.Behavior.myGetInternalFoeLanes(lane_id)

    @classmethod
    def myGetLaneVehicles(cls,lane_id):
        if cls.API_Name=='PanoNewAPI':
            return cls.Behavior.myGetVehicleListInLane(lane_id)
        else:
            return cls.Behavior.myGetLaneVehicles(lane_id)

    @classmethod
    def myGetVehicleLength(cls,car_id):
        return cls.Behavior.myGetVehicleLength(car_id)

    @classmethod
    def myGetLaneLength(cls,lane_id): #上层应用有自己的myGetLaneLength
        return cls.Behavior.myGetLaneLength(lane_id)

    @classmethod
    def myGetVehicleLateralOffset(cls,car_id):
        if cls.API_Name.__eq__('PanoNewAPI'):
            return cls.Behavior.myGetLateralOffset(car_id)
        else:
            return cls.Behavior.myGetVehicleLateralOffset(car_id)

    @classmethod
    def myGetVehicleType(cls,car_id):
        if cls.API_Name.__eq__('InteractionAPI'):
            return cls.Behavior.myGetVehicleType(car_id)
        elif cls.API_Name.__eq__('PanoAPI'):
            return cls.Behavior.myGetVehicleType(car_id).name
        elif cls.API_Name.__eq__('PanoNewAPI'):
            return cls.Behavior.myGetObjectType(car_id)
        else:
            return cls.Behavior.myGetVehicleType(car_id)


    @classmethod
    def myGetVehicleMaxAccel(cls,car_id):
        if cls.API_Name=='PanoNewAPI':
            return cls.Helper.myGetVehicleMaxAccel(car_id)
        else:
            return cls.Behavior.myGetVehicleMaxAccel(car_id)

    @classmethod
    def myGetVehicleMaxDecel(cls,car_id):
        if cls.API_Name=='PanoNewAPI':
            return cls.Helper.myGetVehicleMaxDecel(car_id)
        else:
            return cls.Behavior.myGetVehicleMaxDecel(car_id)

    @classmethod
    def myGetVehicleMaxDecelOrigin(cls,car_id):
        if cls.API_Name=='PanoNewAPI':
            return cls.Helper.myGetVehicleMaxDecel(car_id)
        else:
            raise NotImplementedError

    @classmethod
    def myGetTrafficLightState(cls,car_id, direction): #TODO 未完成
        return cls.Behavior.myGetTrafficLightState(car_id, direction)

    @classmethod
    def myAddVehicle(cls,x, y,current_s, lane_id, speed=0, type=1, shape=0, driver=0):
        if cls.API_Name.__eq__('InteractionAPI'):
            return cls.Behavior.myAddVehicle(x, y, current_s, lane_id,speed,type)
        elif cls.API_Name.__eq__('PanoAPI'):
            return cls.Behavior.myAddVehicle(x, y, speed, type=0, shape=0,driver=0)
        elif cls.API_Name.__eq__('PanoNewAPI'):
            return cls.Behavior.myCreateVehicleXY(lane_id,x, y, speed, type=0, shape=0,driver=0)
        else: #SUMO
            return cls.Behavior.myAddVehicle(x, y, current_s,lane_id, speed=0,type=0, shape=0, driver=0)

    @classmethod
    def myGetLeftLaneIDBasedLane(cls,lane_id):
        if cls.API_Name.__eq__('PanoNewAPI'):
            return cls.Behavior.myGetLeftLaneId(lane_id)
        else:
            return cls.Behavior.myGetLeftLaneIDBasedLane(lane_id)

    @classmethod
    def myGetRightLaneIDBasedLane(cls,lane_id):
        if cls.API_Name.__eq__('PanoNewAPI'):
            return cls.Behavior.myGetRightLaneId(lane_id)
        else:
            return cls.Behavior.myGetRightLaneIDBasedLane(lane_id)

    @classmethod #SUMOtraci使用
    def mySetRoute(cls,veh_id,route):
        if cls.API_Name.__eq__('PanoNewAPI'):
            return cls.Behavior.mySetObjectRouteByLaneList(veh_id,route)
        elif cls.API_Name.__eq__('SumoAPI'):
            return cls.Behavior.mySetRoute(str(veh_id),route)

    @classmethod
    def myTranslateDirectionToRoute(cls, param):
        if cls.API_Name.__eq__('InteractionAPI'):
            log.info('InteractionAPI dont implements myTranslateDirectionToRoute')
        elif cls.API_Name.__eq__('PanoAPI'):
            return cls.Behavior.myTranslateDirectionToRoute(param)
        else:
            log.info('Only PanoAPI implements myTranslateDirectionToRoute')

    @classmethod
    def myTranslateIntTo_next_junction_direction(cls, param):
        if cls.API_Name.__eq__('InteractionAPI'):
            log.info('InteractionAPI dont implements myTranslateIntTo_next_junction_direction')
        elif cls.API_Name.__eq__('PanoAPI'):
            return cls.Behavior.myTranslateIntTo_next_junction_direction(param)
        else:
            log.info('Only PanoAPI implements myTranslateIntTo_next_junction_direction')

    @classmethod
    def isLaneExist(cls, lane):
        if cls.API_Name.__eq__('InteractionAPI'):
            return lane is not None
        elif cls.API_Name.__eq__('PanoAPI'):
            if len(lane) > 2:
                return True
            else:
                return False
        elif cls.API_Name.__eq__('PanoNewAPI'):
            if lane!='':
                return True
            else:
                return False
        else:
            return lane is not None

    @classmethod
    def myGetEdgeIDByLaneID(cls, lane_id):
        return cls.Behavior.myGetEdgeIDByLaneID(lane_id) #只有SUMOAPI有这个功能

    @classmethod
    def myDeleteVehicle(cls,veh_ID):
        if cls.API_Name.__eq__('PanoNewAPI'):
            return cls.Behavior.myDeleteObject(veh_ID)
        return cls.Behavior.myDeleteVehicle(veh_ID)

    @classmethod
    def myMoveObjectByLaneOffset(cls,veh_ID,expOffset, duration = 2.0):
        if cls.API_Name.__eq__('PanoNewAPI'):
            return cls.Behavior.myMoveObjectByLaneOffset(veh_ID,expOffset, duration)

    @classmethod
    def myChangeLane(cls,veh_ID,lane_change_dir):
        if cls.API_Name.__eq__('PanoNewAPI'):
            return cls.Behavior.myChangeLane(veh_ID, lane_change_dir, duration = 2.0)

    @classmethod
    def myGetVehicleIsLaneChanging(cls,veh_ID):
        if cls.API_Name.__eq__('PanoNewAPI'):
            return cls.Behavior.myGetVehicleIsLaneChanging(veh_ID)