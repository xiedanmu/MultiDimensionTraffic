import math
import os

import traci
import sumolib
from numpy import random
import logging
log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())


class SumoAPI:
    net = None# sumolib.net.readNet('hello.net.xml.backup2.backup',withInternal=True) # It will be uesd among several functions

    @classmethod
    def setNetFileInAbPath(cls,dir_str):
        cls.net=sumolib.net.readNet(dir_str,withInternal=True)

    MY_PANOSIM_INVALID_VALUE = -1073741824
    g_nHostVehicleID = 0
    g_dicLaneShapes = {}
    g_dicLaneToLanes = {}
    g_dicLaneFromLanes = {}
    g_dicLaneToFoeLanes = {}

    @classmethod
    def initLaneShapes(cls):
        global g_dicLaneShapes

        global g_dicLaneFromLanes
        global g_dicLaneToLanes
        global g_dicLaneToFoeLanes

        junctionList = cls.myGetJunctionList()
        if len(junctionList) > 0:
            for junctionId in junctionList:
                incomingLanes = cls.myGetIncomingLanes(junctionId)
                for incLaneId in incomingLanes:
                    normalLaneShape = cls.myGetLaneShape(incLaneId)
                    if (len(normalLaneShape) > 1):
                        resizedLaneShape = cls.getLaneShapeWithOnlyXY(normalLaneShape)
                        g_dicLaneShapes[incLaneId] = resizedLaneShape

                    else:
                        log.warning("Warning: [ {} ] has no point list from getLaneShape".format(incLaneId))

                internalLanes = cls.myGetInternalLanes(junctionId)
                for intLaneId in internalLanes:
                    normalLaneShape = cls.myGetLaneShape(intLaneId)
                    log.info("Internal Lanes".format(intLaneId, normalLaneShape))
                    if (len(normalLaneShape) > 1):
                        resizedLaneShape = cls.getLaneShapeWithOnlyXY(normalLaneShape)
                        g_dicLaneShapes[intLaneId] = resizedLaneShape

                    else:
                        log.warning("Warning: [ {} ] has no point list from getLaneShape".format(intLaneId))

        '''# lane to lanes
        for i in g_dicLaneShapes:
            toLanes = []
            lane1Shape = myGetLaneShape(i)
            len1 = len(lane1Shape)
            if(len1 <= 1): continue
            lane1X = lane1Shape[len1-1][0]
            lane1Y = lane1Shape[len1-1][1]
            for j in g_dicLaneShapes:
                if i == j: continue
                lane2Shape = myGetLaneShape(j)
                len2 = len(lane2Shape)
                if(len2<=1): continue
                lane2X = lane2Shape[0][0]
                lane2Y = lane2Shape[0][1]
                if( lane1X == lane2X and lane1Y == lane2Y ):
                    toLanes.append( j )
            if( len(toLanes) ):
                g_dicLaneToLanes[i] = toLanes

        # lane from lanes
        for i in g_dicLaneShapes:
            fromLanes = []
            lane1Shape = myGetLaneShape(i)
            len1 = len(lane1Shape)
            if(len1 <= 1): continue
            lane1X = lane1Shape[0][0]
            lane1Y = lane1Shape[0][1]
            for j in g_dicLaneShapes:
                if i == j: continue
                lane2Shape = myGetLaneShape(j)
                len2 = len(lane2Shape)
                if(len2<=1): continue
                lane2X = lane2Shape[len2-1][0]
                lane2Y = lane2Shape[len2-1][1]
                if( lane1X == lane2X and lane1Y == lane2Y ):
                    fromLanes.append( j )
            if( len(fromLanes) ):
                g_dicLaneFromLanes[i] = fromLanes'''

        # lane to lanes. 这里是忽略中间车道，直接访问下一个车道
        for i in g_dicLaneShapes:
            toLanes = []
            dirs = cls.myGetValidDirections(i)
            if (len(dirs) <= 0): continue
            for dir in dirs:
                toLane = cls.myGetNextLanes(i, dir)
                if toLane != "":
                    toLanes.append(toLane)
            g_dicLaneToLanes[i] = toLanes

        # lane with to-foe lanes
        for i in g_dicLaneShapes:
            foeLanes = []
            isDone = False
            if (i not in g_dicLaneToLanes): continue
            lane1ToLanes = g_dicLaneToLanes[i]
            for j in g_dicLaneShapes:
                if i == j: continue
                if j not in g_dicLaneToLanes: continue
                lane2ToLanes = g_dicLaneToLanes[j]
                for potLane in lane1ToLanes:
                    if potLane in lane2ToLanes:
                        foeLanes.append(j)
                        isDone = True
                        break
                if isDone:
                    break
            if len(foeLanes) > 0:
                g_dicLaneToFoeLanes[i] = foeLanes
        # log.info("Foe Lanes")
        # log.info(g_dicLaneFoeLanes)

    # 函数：myGetLaneToFoeLanes
    # 用途：获得车道去向的敌对车道 不包括相交车道
    # [i]：laneID - 常规车道的ID
    # [o]：[laneID, ..., laneID] - 需要检查的敌对车道ID，仅包括常规车道
    @classmethod
    def myGetLaneToFoeLanes(cls,laneID):
        global g_dicLaneToFoeLanes
        if cls.myCheckInternalLane(laneID): return []
        if laneID not in g_dicLaneToFoeLanes: return []
        return g_dicLaneToFoeLanes[laneID]

    # 函数: myGetCompleteLaneShape
    # 用途: 获得车道点集
    # [i]: laneID - 车道ID
    # [o]: [[x,y,s,yaw], ..., [x,y,s,yaw]] - 点集，带s和yaw
    @staticmethod
    def myGetCompleteLaneShape(laneID):
        if laneID in g_dicLaneShapes:
            return g_dicLaneShapes[laneID]
        else:
            return []

    # 函数: myGetLaneLength
    # 用途: 获得车道长度
    # [i]: laneID - 车道ID
    # [o]: length - 车道长度。 当未找到车道时，返回-1073741824
    @classmethod
    def myGetLaneLengthByLaneShape(cls,laneID):
        shape = cls.myGetCompleteLaneShape(laneID)
        if (len(shape) <= 0): return cls.MY_PANOSIM_INVALID_VALUE
        return shape[len(shape) - 1][2]

    # 函数: myGetVehicleLength
    # 用途: 获得车辆长度
    # [i]: typeId - 车辆类型
    # [o]: float - 车辆长度
    @staticmethod
    def myGetVehicleLengthByTypeID(typeId):
        if typeId == 1:
            return 4.25  # CheryTiggoSUV
        elif typeId == 2:
            return 6.33  # FiatExpressCar
        elif typeId == 3:
            return 5.36  # VolkswagenPickUpCar
        elif typeId == 4:
            return 5.15  # DodgePoliceCar
        elif typeId == 5:
            return 4.55  # InfinitEssence
        elif typeId == 6:
            return 4.82  # LynkCoCar
        elif typeId == 7:
            return 12.2  # MercedesBenzCitarBus
        elif typeId == 8:
            return 9.7  # FordB760SchoolBus
        elif typeId == 9:
            return 3.2  # DaihatsuHijetVan
        elif typeId == 10:
            return 9.22  # ScaniaConcreteTruck
        elif typeId == 11:
            return 8.2  # ScanlaE402FireTruck
        elif typeId == 12:
            return 8.97  # MANTruck
        else:
            return 10  # Error, no this type.

    # 函数: getVehicleWidth
    # 用途: 获得车辆宽度
    # [i]: typeId - 车辆类型
    # [o]: float - 车辆宽度
    @staticmethod
    def myGetVehicleWidthByType(typeId):
        if typeId == 1:
            return 2.1  # CheryTiggoSUV
        elif typeId == 2:
            return 2.64  # FiatExpressCar
        elif typeId == 3:
            return 2.17  # VolkswagenPickUpCar
        elif typeId == 4:
            return 2.04  # DodgePoliceCar
        elif typeId == 5:
            return 2.07  # InfinitEssence
        elif typeId == 6:
            return 1.05  # LynkCoCar
        elif typeId == 7:
            return 2.85  # MercedesBenzCitarBus
        elif typeId == 8:
            return 2.88  # FordB760SchoolBus
        elif typeId == 9:
            return 1.56  # DaihatsuHijetVan
        elif typeId == 10:
            return 3.06  # ScaniaConcreteTruck
        elif typeId == 11:
            return 2.94  # ScanlaE402FireTruck
        elif typeId == 12:
            return 3.47  # MANTruck
        else:
            return 2.1  # Error, no this type.

    # # 函数: getVehicleCenterToFront
    # # 用途: 获得车辆宽度
    # # [i]: typeId - 车辆类型
    # # [o]: float - 车辆中心到车辆前端的距离
    # @staticmethod
    # def getVehicleCenterToFront(typeId):
    #     return 0.0

    # 函数: getXYFromSL
    # 用途: 根据S,L坐标获取目标车道的X,Y坐标
    # [i]: laneID
    # [i]: station 期望的S坐标
    # [i]: offset 期望的L坐标
    # [o]: (x,y) 期望的X,Y坐标
    # [example]: (x,y) = getXYFromSL( getLaneShape(id, -1/0/1), 100, 0 )
    # [o]: s<0 & s>S_max 会返回none值 造成类型报错（使用约束）
    @classmethod
    def getXYFromSL(cls,laneID, station, offset):
        points = cls.myGetCompleteLaneShape(laneID)
        if (len(points) <= 1):
            log.error("Error: No points in laneID {}".format(laneID))
            return (None, None)

        def calculateRatio(points, station):
            ratio = 0
            idx = i = 0

            if station > points[len(points) - 1][2]:
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
                x = xSrc + offset * math.sin(yaw)
                y = ySrc + offset * math.cos(yaw)
                return (x, y)

        x = 0
        y = 0

        pointsWithS = points
        (idx, ratio) = calculateRatio(pointsWithS, station)
        if (idx == None or ratio == None):
            return (None, None)

        if idx == len(pointsWithS) - 1:
            (x, y) = pointOffset(pointsWithS[idx][0], pointsWithS[idx][1], pointsWithS[idx][2], offset)
            return (x, y)

        else:
            (xTmp, yTmp) = linearInterpolation(pointsWithS[idx][0], pointsWithS[idx][1], pointsWithS[idx + 1][0],
                                               pointsWithS[idx + 1][1], ratio)
            (x, y) = pointOffset(xTmp, yTmp, pointsWithS[idx][2], offset)
            return (x, y)

    # 函数: getLaneShapeWithOnlyXY
    # 用途: 根据x,y坐标获取目标车道的x,y,s,yaw坐标  yaw角与GPS保持一致
    # [i]: {[x,y], ..., [x,y]} 传入目标车道道路中间线的信息
    # [o]: [[x,y,s,yaw], ..., [x,y,s,yaw]] 传入目标车道道路中间线的信息
    @staticmethod
    def getLaneShapeWithOnlyXY(points):
        def caculate2pointsYaw(x1, y1, x2, y2):
            yaw = 0
            dy = y2 - y1
            dx = x2 - x1
            if dx == 0 and dy > 0:
                yaw = 0
            if dx == 0 and dy < 0:
                yaw = 180
            if dy == 0 and dx > 0:
                yaw = 90
            if dy == 0 and dx < 0:
                yaw = 270
            if dx > 0 and dy > 0:
                yaw = math.atan(dx / dy) * 180 / math.pi
            elif dx < 0 and dy > 0:
                yaw = 360 + math.atan(dx / dy) * 180 / math.pi
            elif dx < 0 and dy < 0:
                yaw = 180 + math.atan(dx / dy) * 180 / math.pi
            elif dx > 0 and dy < 0:
                yaw = 180 + math.atan(dx / dy) * 180 / math.pi

            return yaw * math.pi / 180

        def caculateStation(x1, y1, x2, y2):
            s = 0
            dy = y2 - y1
            dx = x2 - x1
            s = math.sqrt((dy ** 2) + (dx ** 2))

            return s

        lenLane = len(points)
        i = 0

        if i == 0:
            S_tuple = (0.)
            Yaw_tuple = (caculate2pointsYaw(points[i][0], points[i][1], points[i + 1][0], points[i + 1][1]))
            addedTuple = (S_tuple, Yaw_tuple)
            points[i] = points[i] + addedTuple
            i = i + 1

        while i < lenLane - 1:
            deltaS = caculateStation(points[i - 1][0], points[i - 1][1], points[i][0], points[i][1])
            Yaw = caculate2pointsYaw(points[i][0], points[i][1], points[i + 1][0], points[i + 1][1])
            addedS = points[i - 1][2] + deltaS
            addTuple = (addedS, Yaw)
            points[i] = points[i] + addTuple
            i = i + 1

        if i == lenLane - 1:  # 倒数第一个点与倒数第二个点保持一致
            deltaS = caculateStation(points[i - 1][0], points[i - 1][1], points[i][0], points[i][1])
            Yaw = caculate2pointsYaw(points[i - 1][0], points[i - 1][1], points[i][0], points[i][1])
            addedS = points[i - 1][2] + deltaS
            addTuple = (addedS, Yaw)
            points[i] = points[i] + addTuple

        return points

    # 函数: myGetHostVehicleCoordinate
    # 用途: 获得主车的X,Y坐标
    # [o]: (x,y)
    @classmethod
    def myGetHostVehicleCoordinate(cls):
        return traci.vehicle.getPosition(cls.g_nHostVehicleID)

    @staticmethod
    def myGetVehicleCoordinate(vehID):
        return traci.vehicle.getPosition(str(vehID))

    # 函数: myGetHostVehicleStationInLane
    # 用途: 获得主车在车道中的station
    # [o]: station
    @classmethod
    def myGetHostVehicleStationInLane(cls):
        # return getDistanceFromLastJunction(g_nHostVehicleID)  # v9
        return traci.vehicle.getLanePosition(cls.g_nHostVehicleID)

    # 函数: myGetHostVehicleSpeed
    # 用途: 获得主车的车速 [m/s]
    # [o]: speed
    @classmethod
    def myGetHostVehicleSpeed(cls):
        return traci.vehicle.getSpeed(cls.g_nHostVehicleID)

    # 函数: myCheckHostVehicle
    # 用途: 判断是否为主车
    # [i]: vehID - 车辆ID
    # [o]: Bool - 是否为主车
    @staticmethod
    def myCheckHostVehicle(vehID):
        if vehID == 0:
            return True
        else:
            return False

    # # 函数: myCheckLaneExist
    # # 用途: 判断车辆是否存在左车道或右车道
    # # [i]: vehID - 车辆ID
    # # [i]: dir -   -1：左侧；  1右侧；
    # # [o]: Bool - True有车道，False无车道
    # @staticmethod
    # def myCheckLaneExist(vehID, dir):
    #     if (vehID < 0):
    #         return False
    #
    #     tmpID = 0
    #     if dir == 0:
    #         return False
    #     elif dir < 0:  # 左侧
    #         tmpID = getLeftLeaderVehicle(vehID)
    #         log.info("GetLeftLeader".format(tmpID))
    #         if (tmpID >= 0) or (tmpID == -1):
    #             return True
    #         else:
    #             return False
    #     else:  # 右侧
    #         tmpID = getRightLeaderVehicle(vehID)
    #         if (tmpID >= 0) or (tmpID == -1):
    #             return True
    #         else:
    #             return False

    @staticmethod
    def myGetLeftLeaderVehicle(vehID):
        left_leaders=traci.vehicle.getLeftLeaders(str(vehID))
        if left_leaders is None or len(left_leaders)==0:
            return None
        elif type(left_leaders[0]) is not str:
            return left_leaders[0][0]#(("car100", 10.00),("car101",12.01))
        else:
            return left_leaders[0]

    @staticmethod
    def myGetRightLeaderVehicle(vehID):
        right_leaders = traci.vehicle.getRightLeaders(str(vehID))
        if right_leaders is None or len(right_leaders)==0:
            return None
        elif type(right_leaders[0]) is not str:
            return right_leaders[0][0]  # (("car100", 10.00),("car101",12.01))
        else:
            return right_leaders[0]

    @staticmethod
    def myGetLeftFollowerVehicle(vehID):
        left_follower = traci.vehicle.getLeftFollowers(str(vehID))
        if left_follower is None or len(left_follower)==0:
            return None
        elif type(left_follower[0]) is not str:
            return left_follower[0][0]  # (("car100", 10.00),("car101",12.01))
        else:
            return left_follower[0]

    @staticmethod
    def myGetRightFollowerVehicle(vehID):
        right_follower = traci.vehicle.getRightFollowers(str(vehID))
        if right_follower is None or len(right_follower)==0:
            return None
        elif type(right_follower[0]) is not str:
            return right_follower[0][0]  # (("car100", 10.00),("car101",12.01))
        else:
            return right_follower[0]

    # @staticmethod
    # def myGetRoute(vehID):
    #     return getRoute(vehID)

    @staticmethod
    def myGetVehicleList():
        #return list(traci.vehicle.getIDList())
        return list(map(lambda x:int(x), traci.vehicle.getIDList()))  # API返回的是一个tuple,所以要转换成list，且每个元素变为int

    @staticmethod
    # API v10
    def myGetDistanceToLaneEnd(vehID):
        lane_id=traci.vehicle.getLaneID(str(vehID))
        current_lane_length=traci.lane.getLength(lane_id)
        distance_to_lane_end=current_lane_length-traci.vehicle.getLanePosition(str(vehID))
        return distance_to_lane_end

    @staticmethod
    def myGetDistanceFromLaneStart(vehID):
        return traci.vehicle.getLanePosition(str(vehID))

    @staticmethod
    def myGetLaneID(vehID):
        return traci.vehicle.getLaneID(str(vehID))

    @classmethod
    def myGetLeftLaneID(cls,vehID):
        current_lane_id = cls.myGetLaneID(str(vehID))
        if SumoAPI.myCheckInternalLane(current_lane_id):
            return None
        current_edge_id = traci.lane.getEdgeID(current_lane_id)
        current_edge = cls.net.getEdge(current_edge_id)
        max_lane_num = current_edge.getLaneNumber()

        lane = cls.net.getLane(current_lane_id)
        lane_index = lane.getIndex()
        left_lane_index = lane_index + 1

        if left_lane_index >= max_lane_num or left_lane_index < 0:
            return None
        target_lane = current_edge.getLane(left_lane_index)
        return target_lane.getID()

    @classmethod
    def myGetLeftLaneIDBasedLane(cls,current_lane_id):
        if SumoAPI.myCheckInternalLane(current_lane_id):
            return None
        current_edge_id = traci.lane.getEdgeID(current_lane_id)
        current_edge = cls.net.getEdge(current_edge_id)
        max_lane_num = current_edge.getLaneNumber()

        lane = cls.net.getLane(current_lane_id)
        lane_index = lane.getIndex()
        left_lane_index = lane_index + 1

        if left_lane_index >= max_lane_num or left_lane_index < 0:
            return None
        target_lane = current_edge.getLane(left_lane_index)
        return target_lane.getID()

    @classmethod
    def myGetRightLaneID(cls,vehID):
        current_lane_id=cls.myGetLaneID(str(vehID))
        if SumoAPI.myCheckInternalLane(current_lane_id):
            return None
        current_edge_id=traci.lane.getEdgeID(current_lane_id)
        current_edge=cls.net.getEdge(current_edge_id)
        max_lane_num=current_edge.getLaneNumber()

        lane=cls.net.getLane(current_lane_id)
        lane_index=lane.getIndex()
        right_lane_index=lane_index-1

        if right_lane_index>=max_lane_num or right_lane_index<0:
            return None
        target_lane=current_edge.getLane(right_lane_index)
        return target_lane.getID()

    @classmethod
    def myGetRightLaneIDBasedLane(cls,current_lane_id):
        if SumoAPI.myCheckInternalLane(current_lane_id):
            return None
        current_edge_id = traci.lane.getEdgeID(current_lane_id)
        current_edge = cls.net.getEdge(current_edge_id)
        max_lane_num = current_edge.getLaneNumber()

        lane = cls.net.getLane(current_lane_id)
        lane_index = lane.getIndex()
        right_lane_index = lane_index - 1

        if right_lane_index >= max_lane_num or right_lane_index < 0:
            return None
        target_lane = current_edge.getLane(right_lane_index)
        return target_lane.getID()

    # @staticmethod
    # def myGetVehiclesInLane(laneID):
    #     return getLaneVehicles(laneID)

    @staticmethod
    def myGetLaneShape(laneID):
        return traci.lane.getShape(laneID)

    # @classmethod
    # def myGetPossibleDirections(cls,vehID):
    #     laneID = cls.myGetLaneID(vehID)
    #     return getValidDirections(laneID)

    @staticmethod
    def myCheckInternalLane(laneID):
        if laneID.startswith(':'):
            return True
        else:
            return False

    @classmethod
    def myGetValidDirections(cls,laneID):
        links=traci.lane.getLinks(laneID)
        directions=list()
        for link in links:
            current_direction = link[6]
            if current_direction not in directions:
                directions.append(current_direction)
        return directions

    @staticmethod
    def myGetNextLanes(laneID, dir): #弃用
        links = traci.lane.getLinks(laneID)
        toLanes=list()
        for link in links:
            if dir==link[6]:
                if link[4] != '':
                    toLanes.append(link[4]) #4 是指的internal lane
                if link[0] != '':
                    toLanes.append(link[0]) #0 是指的经过internal lane 后面在EDGE上的lane
                break #只取其中一个
        return toLanes

    @classmethod
    def myGetAllNextLanes(cls,laneID, dir):
        links = traci.lane.getLinks(laneID)
        toLanes=list()
        for link in links:
            next_lane=[]
            if dir==link[6]:
                if link[4]!='':
                    next_lane.append(link[4])
                    next_next_lane=cls.myGetNextLaneRecursion(link[4])
                    if next_next_lane is not None:
                        next_lane.extend(next_next_lane)

                if link[0]!='':
                    next_lane.append(link[0])
                toLanes.append(next_lane)
        return toLanes

    @classmethod
    def myGetNextLaneRecursion(cls,laneID):
        links = traci.lane.getLinks(laneID)
        next_lane=[]
        for link in links:
            if link[4] != '':
                next_lane.append(link[4])
                next_next_lane = cls.myGetNextLaneRecursion(link[4])
                if next_next_lane is not None:
                    next_lane.extend(next_next_lane)
        if len(next_lane)>0:
            return next_lane
        else:
            pass
    # @staticmethod
    # def myGetToJunction(laneID):
    #     return getToJunction(laneID)

    @staticmethod
    def myGetJunctionList():
        junction_list = traci.junction.getIDList()
        temp_list = []
        for junction in junction_list:
            if junction.startswith(":"):
                continue
            temp_list.append(junction)
        return temp_list

    @classmethod
    def myGetIncomingLanes(cls,junctionId):
        targetNode=cls.net.getNode(junctionId)
        incoming_lanes = targetNode._incLanes
        return incoming_lanes

        # incoming_lanes=targetNode._incLanes
        # if incoming_lanes[0] == '':
        #     return []
        # else:
        #     return incoming_lanes

        # incoming_edges=targetNode.getIncoming()
        # for incoming_edge in incoming_edges:
        #     lanes=incoming_edge.getLanes()
        #     for lane in lanes:
        #         lane_id=lane.getID()
        #         result.append(lane_id)
        # return result

    @classmethod
    def myGetInternalLanes(cls,junctionId):
        targetNode = cls.net.getNode(junctionId)
        internal_lanes=[]
        for edge in cls.net.getEdges():
            if edge.getFromNode() == targetNode and edge.getToNode() == targetNode:
                lanes=edge.getLanes()
                for lane in lanes:
                    internal_lanes.append(lane.getID())
        return internal_lanes
        # targetNode = cls.net.getNode(junctionId)
        # result=targetNode.getInternal()
        # if len(result)==1 and result[0]=='': #和deadend相链接的junction返回['']这种list
        #     return None
        # return result

    # @staticmethod
    # def myGetLongitudinalDistance(id1, id2):  # 这个结果时id1 - id2?还是id2 - id1?  如果两车在不同的edge，返回什么？
    #     return getLongitudinalDistance(id1, id2)

    # @staticmethod
    # def myGetLateralDistance(id1, id2): #不需要使用
    #     return getLateralDistance(id1, id2)

    @staticmethod
    def myGetVehicleSpeed(vehID):
        return traci.vehicle.getSpeed(str(vehID))

    @staticmethod
    def myGetVehicleX(vehID):
        return traci.vehicle.getPosition(str(vehID))[0]

    @staticmethod
    def myGetVehicleY(vehID):
        return traci.vehicle.getPosition(str(vehID))[1]

    @staticmethod
    def myGetVehicleYaw(vehID):  # 返回弧度rad
        def myDeg2Rad(degree):
            return degree * math.pi / 180

        yaw = traci.vehicle.getAngle(str(vehID))
        return myDeg2Rad(-yaw) + math.pi / 2  # 这里是从SUMO坐标系（正北，顺时针为正），转换为（正东，逆时针为正）

    @classmethod
    def myMoveTo(cls,vehID, x, y, yaw,lane,s,l):
        # position = traci.vehicle.getPosition(vehID)
        try:
            # if vehID=='m2.0':
            #     if y < 826:
            #         traci.vehicle.setColor(vehID,(255,0,0))
            #         route=traci.vehicle.getRoute(vehID)
            #         traci.vehicle.setRoute(vehID,['-L5','m4'])
            #         log.info('old route:',route)
            #         route = traci.vehicle.getRoute(vehID)
            #         log.info('new route:',route)
            result = traci.simulation.convertRoad(x,y) #返回来的参数第一个是edgeID，第二个是s值，第三个是index
            #edgeId=traci.vehicle.getRoadID(vehID)
            traci.vehicle.moveToXY(vehID=str(vehID),x=x,y=y,edgeID=result[0],lane=result[2],angle=yaw,keepRoute=0)
        except:

            lane_id=cls.myGetLaneID(str(vehID))
            log.info('lane id is:',lane_id,'of vehicle:',vehID,'move to not work,pls check')
        #
        # if (cls.myIsDeadEnd(curLaneID)):
        #     station = cls.myGetDistanceFromLaneStart(vehID)
        #     if (cls.myGetLaneLength(curLaneID) - station < 5):
        #         return
        #
        # degYaw = yaw * 180 / math.pi
        # moveTo(vehID, x, y, degYaw)

    # @staticmethod
    # def my_Move_To(vehID, x, y, yaw):
    #     moveTo(vehID, x, y, yaw)

    # # 函数: myCoorTransFromVehicleToSUMOWorld
    # # 用途: 将车体坐标系下的轨迹，转换到世界坐标系下
    # # [i] 车体坐标系下坐标 [ [x1,y1,yaw1,speed1,t1], ..., [xn,yn,yawn,speedn,tn] ]  注意，此处传入的yaw为rad；时间为s，以0开始；速度为m/s
    # # [o] SUMO世界坐标系 [ [x1,y1,yaw1,speed1,t1], ..., [xn,yn,yawn,speedn,tn] ]  返回yaw为rad；时间为ms；速度为m/s
    # @staticmethod
    # def myCoorTransFromVehicleToSUMOWorld(curX, curY, curYaw, curTime, traj_in_vehicle):
    #     def myCoorTrans(old_x, old_y, new_in_old_x, new_in_old_y, new_from_old_theta):
    #         new_x = (old_x - new_in_old_x) * math.cos(new_from_old_theta) + (old_y - new_in_old_y) * math.sin(
    #             new_from_old_theta)
    #         new_y = (old_y - new_in_old_y) * math.cos(new_from_old_theta) - (old_x - new_in_old_x) * math.sin(
    #             new_from_old_theta)
    #         return (new_x, new_y)
    #
    #     tran_x, trans_y = myCoorTrans(0, 0, curX, curY, curYaw)
    #     tran_yaw = -curYaw
    #     traj_in_world = []
    #     for e in traj_in_vehicle:
    #         (newX, newY) = myCoorTrans(e[0], e[1], tran_x, trans_y, tran_yaw)
    #         newYaw = math.pi / 2 - (e[2] + curYaw)  # e[2] + math.pi / 2
    #         newSpeed = e[3]
    #         newTime = e[4] * 1000 + curTime
    #         traj_in_world.append([newX, newY, newYaw, newSpeed, newTime])
    #     return traj_in_world
    #
    # @classmethod
    # def mySetValidRoute(cls,vehID, log.infoFlag=False):
    #     laneID = cls.myGetLaneID(vehID)
    #     if (laneID == ""): return False
    #     potentialDirs = cls.myGetValidDirections(laneID)
    #     dir = 0
    #     if (len(potentialDirs) > 0):
    #         dir = potentialDirs[int(cls.myGetRandomDouble(0, len(potentialDirs) - 1))]
    #     cls.myChangeRoute(vehID, cls.route_type(dir))
    #     if (log.infoFlag == True):
    #         log.info("FromLane:", laneID, "Valid Dirs:", potentialDirs)
    #         log.info("SetRoute", dir)
    #     return True
    #
    # @staticmethod
    # def myGetRandomDouble(min, max):
    #     rand = random.ranf()
    #     return min + rand * (max - min)

    '''
    # API BXS新增
    def myGetOffset(vehID):
        return getVehicleLateralOffset(vehID)


    def myConflict(ConflictPoint, junctionID):
        ConflictPointLane = [['gneE0_0', ':gneJ1_1_0'], ['gneE3_0', ':gneJ1_0_0']]  # todo
        vehList = getVehicleList()
        vehList1 = getLaneVehicles(ConflictPointLane[0][0])
        for i in vehList1:
            if myGetDistanceToLaneEnd(i) > 20:
                pp = vehList1.index(i)
                vehList.pop(pp)
        vehList2 = []
        vehList4 = []
        for i in vehList:
            if myGetVehicleX(i) > 57.6 and myGetVehicleX(i) < ConflictPoint[0][0]:  # #todo, 需要接口函数判断节点边界
                if myGetLaneID(i) == ':gneJ1_1_0':  # todo
                    vehList2.append(i)
                elif myGetLaneID(i) == ':gneJ1_0_0':  # todo
                    vehList4.append(i)
        vehList3 = getLaneVehicles(ConflictPointLane[1][0])
        for i in vehList3:
            if myGetDistanceToLaneEnd(i) > 20:
                pp = vehList3.index(i)
                vehList.pop(pp)

        if vehList1 and vehList3:
            ConflictPointQueue = [[[vehList1[-1]], vehList2], [[vehList3[-1]], vehList4]]
        elif vehList1:
            ConflictPointQueue = [[[vehList1[-1]], vehList2], [[vehList3], vehList4]]
        elif vehList3:
            ConflictPointQueue = [[vehList1, vehList2], [[vehList3[-1]], vehList4]]
        else:
            ConflictPointQueue = [[vehList1, vehList2], [[vehList3], vehList4]]

        # [[[9], []], [[7], []]]
        # [[9], [7]]
        ConflictPointQueueFin1 = []
        ConflictPointQueueFin2 = []

        if ConflictPointQueue[0][0]:
            ConflictPointQueueFin1.append(ConflictPointQueue[0][0][0])
        if ConflictPointQueue[0][1]:
            dis = {}
            for i in ConflictPointQueue[0][1]:
                dis[i] = myGetVehicleX(i)
            dis = sorted(dis.items(), key=lambda item: item[1])
            tem = []
            for i in range(len(dis)):
                tem.append(dis[i][0])
            for i in tem:
                ConflictPointQueueFin1.append(i)

        if ConflictPointQueue[1][0]:
            ConflictPointQueueFin2.append(ConflictPointQueue[1][0][0])
        if ConflictPointQueue[1][1]:
            dis = {}
            for i in ConflictPointQueue[1][1]:
                dis[i] = myGetVehicleX(i)
            dis = sorted(dis.items(), key=lambda item: item[1])
            tem = []
            for i in range(len(dis)):
                tem.append(dis[i][0])
            for i in tem:
                ConflictPointQueueFin2.append(i)

        ConflictPointQueueFin = [ConflictPointQueueFin1, ConflictPointQueueFin2]

        # log.info('2222',ConflictPointQueue)
        # log.info('6666',ConflictPointQueueFin)
        # for i in ConflictPointQueueFin[0]:
        #     log.info("7777",myGetVehicleX(i))
        # for i in ConflictPointQueueFin[1]:
        #     log.info("8888", myGetVehicleX(i))

        # 获取冲突点前前的两个车队序列【0：主干道序列，1：匝道系列】
        return ConflictPointQueueFin


    def follow(vehID):
        ownX = myGetVehicleX(vehID)
        ownY = myGetVehicleY(vehID)
        vehList = getVehicleList()
        for i in vehList:
            if i is not vehID:
                x = myGetVehicleX(i)
                y = myGetVehicleY(i)
                dis = math.sqrt()
        speed = 10
        return speed


    def myFindLeader():
        pass


    def myIDM(vehID, leadVehID, deltaDistance = -1):
        curX = myGetVehicleX(vehID)
        curSpeed = myGetVehicleSpeed(vehID)
        vDesired = 120 / 3.6
        delta = 4
        thwDesired = 1.5
        s0 = 10
        aMax = 2
        bCom = 2
        if leadVehID < 0:
            acc = aMax * (1. - pow((curSpeed / vDesired), delta))
        else:
            curLeadX = myGetVehicleX(leadVehID)
            deltaS = 0
            if deltaDistance > 0:
                deltaS = deltaDistance
            else:
                deltaS = curLeadX - curX
            deltaS = deltaS - getVehicleLength(leadVehID)
            leadSpeed = myGetVehicleSpeed(leadVehID)
            deltaV = curSpeed - leadSpeed
            if deltaS < 0.5:
                deltaS = 0.5

            sStar = s0 + curSpeed * thwDesired + curSpeed * deltaV / (2. * pow(aMax * bCom, 0.5))
            acc = aMax * (1. - pow((curSpeed / vDesired), delta) - pow(sStar / deltaS, 2.))
        return acc
    '''

    """新增的封装函数"""

    @staticmethod
    def myGetVehicleLane(car_id):
        return traci.vehicle.getLaneID(str(car_id))

    @staticmethod
    def myGetVehicleAccel(car_id):
        return traci.vehicle.getAcceleration(str(car_id))

    @staticmethod
    def myGetLeaderVehicle(car_id):
        try: #刚开始会出现    raise TraCIException(err, prefix[1], _RESULTS[prefix[2]]) KeyError: 104这种错误
            leader_list = traci.vehicle.getLeader(str(car_id))
        except:
            return None
        else:
            if leader_list is None:
                return None
            if type(leader_list[0]) is not str:
                return leader_list[0][0]  # (("car100", 10.00),("car101",12.01))
            else:
                return leader_list[0]
    @staticmethod
    def myChangeSpeed(car_id, current_speed, duration):
        # try:
        #     traci.vehicle.setSpeed(car_id, current_speed)
        # except:
        #     log.info(car_id+' change speed fail')
        traci.vehicle.setSpeed(str(car_id), current_speed)

    @staticmethod
    def myGetVehicleRoute(id):
        return traci.vehicle.getRoute(str(id))

    @staticmethod
    def myChangeRoute(id, myLaneRute):
        traci.vehicle.setRoute(str(id),myLaneRute)

    @staticmethod
    def myGetVehicleMaxSpeed(id):
        return traci.vehicle.getAllowedSpeed(str(id))

    @staticmethod
    def myIsDeadEnd(lane_id) -> bool:
        link_num = traci.lane.getLinkNumber(lane_id)
        if link_num==0:
            return True
        else:
            return False

    # @staticmethod
    # def myDeleteVehicle(id):
    #     return deleteVehicle(id)

    @classmethod
    def myGetInternalFoeLanes(cls,lane_id): #TODO 这个地方返回的格式和Pano不一样 需要进一步修改
        foe_lanes=traci.lane.getInternalFoes(lane_id)
        foe_lanes_and_type=[]
        for foe_lane in foe_lanes:
            foe_lanes_and_type.append((foe_lane,'conflict'))
        return foe_lanes_and_type # 返回[(lane, type)]返回与该内部lane冲突的所有lane和冲突类型（Merging/MergingAdjacent/Crossing）

    @staticmethod
    def myGetLaneVehicles(lane_id):
        return map(lambda x: int(x), traci.lane.getLastStepVehicleIDs(lane_id))
        #return traci.lane.getLastStepVehicleIDs(lane_id)

    @staticmethod
    def myGetVehicleLength(car_id):
        return traci.vehicle.getLength(str(car_id))

    @staticmethod
    def myGetLaneLength(lane_id):
        return traci.lane.getLength(lane_id)

    @staticmethod
    def myGetVehicleLateralOffset(car_id):
        return traci.vehicle.getLateralLanePosition(str(car_id))

    @staticmethod
    def myGetVehicleType(car_id):
        return traci.vehicle.getTypeID(str(car_id)) # 返回'CarA','DEFAULT_TYPE'之类的

    @staticmethod
    def myGetVehicleMaxAccel(car_id):
        return traci.vehicle.getAccel(str(car_id))

    @staticmethod
    def myGetVehicleMaxDecel(car_id):
        return traci.vehicle.getDecel(str(car_id))

    @classmethod
    def myGetTrafficLightState(id, direction): #TODO 未完成
        traci.trafficlight.getControlledLinks()
        return getTrafficLightState(id, direction)

    auto_increment_veh_id=1
    @classmethod
    def myAddVehicle(cls,x, y,current_s,lane_id, speed=0, type=0, shape=0, driver=0): #TODO 未完成
        try:
            result = traci.simulation.convertRoad(x, y)  # 返回来的参数第一个是edgeID，第二个是s值，第三个是index
            edgeID = result[0]
            edges = list()
            edges.append(edgeID)
            lane_index = cls.net.getLane(lane_id).getIndex()
            route_id = str(edgeID)
            if route_id not in traci.route.getIDList():
                traci.route.add(routeID=route_id, edges=edges)
            traci.vehicle.add(vehID=str(cls.auto_increment_veh_id), routeID=route_id, departLane=lane_index,
                              departPos=current_s,departSpeed=str(speed))
            cls.auto_increment_veh_id += 1
        except:
            log.info('myAddVehicle wrong')

    @classmethod
    def mySetRoute(cls,veh_id, route):
        route_id=str(route)
        if route_id not in traci.route.getIDList():
            traci.route.add(routeID=route_id, edges=route)
        traci.vehicle.setRoute(str(veh_id),route)

        # temp_route_id=str(['-L5',':c1_0','L6'])
        # traci.route.add(routeID=temp_route_id, edges=['-L5',':c1_0','L6'])
        # traci.vehicle.setRoute(veh_id, ['-L5',':c1_0','L6'])

    @classmethod
    def myGetEdgeIDByLaneID(cls,lane_id):
        lane=cls.net.getLane(lane_id)
        edge_id=lane.getEdge().getID()
        return edge_id

    @staticmethod
    def myGetTime():
        return traci.simulation.getTime()

    @classmethod
    def myDeleteVehicle(cls,veh_id):
        return traci.vehicle.remove(str(veh_id))
