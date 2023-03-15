#对应PanoSimAPI的文件

import math
from numpy import random
import logging

from Library.package_platformAPI.ExternalInterfaceImpl import ExternalInterfaceImpl

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())

class InteractionAPI:
    externalInterface=ExternalInterfaceImpl

    MY_PANOSIM_INVALID_VALUE = -1073741824
    g_nHostVehicleID = 0
    g_dicLaneShapes = {}
    g_dicLaneToLanes = {}
    g_dicLaneFromLanes = {}
    g_dicLaneToFoeLanes = {}

    # @classmethod
    # def initLaneShapes(cls):
    #     global g_dicLaneShapes
    #
    #     global g_dicLaneFromLanes
    #     global g_dicLaneToLanes
    #     global g_dicLaneToFoeLanes
    #
    #     junctionList = cls.myGetJunctionList()
    #     if len(junctionList) > 0:
    #         for junctionId in junctionList:
    #             incomingLanes = cls.myGetIncomingLanes(junctionId)
    #             for incLaneId in incomingLanes:
    #                 normalLaneShape = cls.myGetLaneShape(incLaneId)
    #                 if (len(normalLaneShape) > 1):
    #                     resizedLaneShape = cls.getLaneShapeWithOnlyXY(normalLaneShape)
    #                     g_dicLaneShapes[incLaneId] = resizedLaneShape
    #
    #                 else:
    #                     log.warning("Warning: [ {} ] has no point list from getLaneShape".format(incLaneId))
    #
    #             internalLanes = cls.myGetInternalLanes(junctionId)
    #             for intLaneId in internalLanes:
    #                 normalLaneShape = cls.myGetLaneShape(intLaneId)
    #                 log.info("Internal Lanes".format(intLaneId, normalLaneShape))
    #                 if (len(normalLaneShape) > 1):
    #                     resizedLaneShape = cls.getLaneShapeWithOnlyXY(normalLaneShape)
    #                     g_dicLaneShapes[intLaneId] = resizedLaneShape
    #
    #                 else:
    #                     log.warning("Warning: [ {} ] has no point list from getLaneShape".format(intLaneId))
    #
    #     '''# lane to lanes
    #     for i in g_dicLaneShapes:
    #         toLanes = []
    #         lane1Shape = myGetLaneShape(i)
    #         len1 = len(lane1Shape)
    #         if(len1 <= 1): continue
    #         lane1X = lane1Shape[len1-1][0]
    #         lane1Y = lane1Shape[len1-1][1]
    #         for j in g_dicLaneShapes:
    #             if i == j: continue
    #             lane2Shape = myGetLaneShape(j)
    #             len2 = len(lane2Shape)
    #             if(len2<=1): continue
    #             lane2X = lane2Shape[0][0]
    #             lane2Y = lane2Shape[0][1]
    #             if( lane1X == lane2X and lane1Y == lane2Y ):
    #                 toLanes.append( j )
    #         if( len(toLanes) ):
    #             g_dicLaneToLanes[i] = toLanes
    #
    #     # lane from lanes
    #     for i in g_dicLaneShapes:
    #         fromLanes = []
    #         lane1Shape = myGetLaneShape(i)
    #         len1 = len(lane1Shape)
    #         if(len1 <= 1): continue
    #         lane1X = lane1Shape[0][0]
    #         lane1Y = lane1Shape[0][1]
    #         for j in g_dicLaneShapes:
    #             if i == j: continue
    #             lane2Shape = myGetLaneShape(j)
    #             len2 = len(lane2Shape)
    #             if(len2<=1): continue
    #             lane2X = lane2Shape[len2-1][0]
    #             lane2Y = lane2Shape[len2-1][1]
    #             if( lane1X == lane2X and lane1Y == lane2Y ):
    #                 fromLanes.append( j )
    #         if( len(fromLanes) ):
    #             g_dicLaneFromLanes[i] = fromLanes'''
    #
    #     # lane to lanes. 这里是忽略中间车道，直接访问下一个车道
    #     for i in g_dicLaneShapes:
    #         toLanes = []
    #         dirs = cls.myGetValidDirections(i)
    #         if (len(dirs) <= 0): continue
    #         for dir in dirs:
    #             toLane = cls.myGetNextLanes(i, dir)
    #             if toLane != "":
    #                 toLanes.append(toLane)
    #         g_dicLaneToLanes[i] = toLanes
    #
    #     # lane with to-foe lanes
    #     for i in g_dicLaneShapes:
    #         foeLanes = []
    #         isDone = False
    #         if (i not in g_dicLaneToLanes): continue
    #         lane1ToLanes = g_dicLaneToLanes[i]
    #         for j in g_dicLaneShapes:
    #             if i == j: continue
    #             if j not in g_dicLaneToLanes: continue
    #             lane2ToLanes = g_dicLaneToLanes[j]
    #             for potLane in lane1ToLanes:
    #                 if potLane in lane2ToLanes:
    #                     foeLanes.append(j)
    #                     isDone = True
    #                     break
    #             if isDone:
    #                 break
    #         if len(foeLanes) > 0:
    #             g_dicLaneToFoeLanes[i] = foeLanes
    #     # print("Foe Lanes")
    #     # print(g_dicLaneFoeLanes)

    # 函数：myGetLaneToFoeLanes
    # 用途：获得车道去向的敌对车道 todo:不包括相交车道
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
    # @staticmethod
    # def myGetCompleteLaneShape(laneID):
    #     if laneID in g_dicLaneShapes:
    #         return g_dicLaneShapes[laneID]
    #     else:
    #         return []

    # 函数: myGetLaneLength
    # 用途: 获得车道长度
    # [i]: laneID - 车道ID
    # [o]: length - 车道长度。 当未找到车道时，返回-1073741824
    # @classmethod
    # def myGetLaneLength(cls,laneID):
    #     shape = cls.myGetCompleteLaneShape(laneID)
    #     if (len(shape) <= 0): return cls.MY_PANOSIM_INVALID_VALUE
    #     return shape[len(shape) - 1][2]

    # 函数: myGetVehicleLength
    # 用途: 获得车辆长度
    # [i]: typeId - 车辆类型
    # [o]: float - 车辆长度
    # @staticmethod
    # def myGetVehicleLength(typeId):
    #     if typeId == 1:
    #         return 4.25  # CheryTiggoSUV
    #     elif typeId == 2:
    #         return 6.33  # FiatExpressCar
    #     elif typeId == 3:
    #         return 5.36  # VolkswagenPickUpCar
    #     elif typeId == 4:
    #         return 5.15  # DodgePoliceCar
    #     elif typeId == 5:
    #         return 4.55  # InfinitEssence
    #     elif typeId == 6:
    #         return 4.82  # LynkCoCar
    #     elif typeId == 7:
    #         return 12.2  # MercedesBenzCitarBus
    #     elif typeId == 8:
    #         return 9.7  # FordB760SchoolBus
    #     elif typeId == 9:
    #         return 3.2  # DaihatsuHijetVan
    #     elif typeId == 10:
    #         return 9.22  # ScaniaConcreteTruck
    #     elif typeId == 11:
    #         return 8.2  # ScanlaE402FireTruck
    #     elif typeId == 12:
    #         return 8.97  # MANTruck
    #     else:
    #         return 10  # Error, no this type.

    # 函数: getVehicleWidth
    # 用途: 获得车辆宽度
    # [i]: typeId - 车辆类型
    # [o]: float - 车辆宽度
    @staticmethod
    def getVehicleWidth(typeId):
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

    # 函数: getVehicleCenterToFront
    # 用途: 获得车辆宽度
    # [i]: typeId - 车辆类型
    # [o]: float - 车辆中心到车辆前端的距离
    @staticmethod
    def getVehicleCenterToFront(typeId):
        return 0.0

    # 函数: getVehicleCenterToBack
    # 用途: 获得车辆宽度
    # [i]: typeId - 车辆类型
    # [o]: float - 车辆中心到车辆后端的距离
    @classmethod
    def getVehicleCenterToBack(cls,typeId):
        return cls.externalInterface.getVehicleLengthById(typeId)

    @classmethod
    def getVehicleCenterToBackById(cls,vehicle_id):
        return cls.externalInterface.getVehicleLengthById(vehicle_id)

    # 函数: getVehicleModelingControlRights
    # 用途: 判断车辆模式是否有控制权。该函数是一个临时函数，
    #      用于判断是否接近junction，当模型不具备junction的行为时，
    #      可以将控制权交还SUMO模型。
    # [i]: vehId - 车辆ID
    # [o]: int - True：可以由交通模型控制；   False：交还SUMO模型控制
    @classmethod
    def getVehicleModelingControlRights(cls,vehId):
        # if( vehId < 0 ):
        #     return False
        # curLane = myGetLaneID(vehId)
        # if( myCheckInternalLane(curLane) ):
        #     return False
        disToJunction = cls.myGetDistanceToLaneEnd(vehId)
        disFromJunction = cls.myGetDistanceFromLaneStart(vehId)
        vehSpeed = cls.externalInterface.getVehicleSpeed(vehId)
        timeToJunction = 9999

        if (vehSpeed > 0.1):
            timeToJunction = disToJunction / vehSpeed

        if (disToJunction < 20 and disToJunction >= 0):  # or (timeToJunction < 2.5 and timeToJunction >= 0):
            return False
        # elif disFromJunction < 20:
        #     return False
        if cls.myCheckInternalLane(cls.myGetLaneID(vehId)):
            return False
        else:
            return True

    # 函数: getXYFromSL
    # 用途: 根据S,L坐标获取目标车道的X,Y坐标
    # [i]: laneID
    # [i]: station 期望的S坐标
    # [i]: offset 期望的L坐标
    # [o]: (x,y) 期望的X,Y坐标
    # [example]: (x,y) = getXYFromSL( getLaneShape(id, -1/0/1), 100, 0 )
    # [o]: s<0 & s>S_max 会返回none值 造成类型报错（使用约束）
    # @classmethod #TODO 应该可以删除了
    # def getXYFromSL(cls,laneID, station, offset):
    #     points = cls.myGetCompleteLaneShape(laneID)
    #     if (len(points) <= 1):
    #         log.error("Error: No points in laneID {}".format(laneID))
    #         return (None, None)
    #
    #     def calculateRatio(points, station):
    #         ratio = 0
    #         idx = i = 0
    #
    #         if station > points[len(points) - 1][2]:
    #             idx = None
    #             ratio = None
    #
    #         elif station < 0:
    #             idx = None
    #             ratio = None
    #
    #         elif station == points[len(points) - 1][2]:
    #             idx = len(points) - 1
    #             ratio = 1
    #
    #         else:
    #             while i < len(points) - 1:
    #                 if (station >= points[i][2] and station < points[i + 1][2]):
    #                     idx = i
    #                     break
    #                 else:
    #                     i = i + 1
    #
    #         if idx != None and idx != len(points) - 1:
    #             ratio = (station - points[idx][2]) / (points[idx + 1][2] - points[idx][2])
    #         return (idx, ratio)
    #
    #     def linearInterpolation(x1, y1, x2, y2, ratio):
    #         if ratio == None:
    #             return (None, None)
    #         else:
    #             x = x1 + ratio * (x2 - x1)
    #             y = y1 + ratio * (y2 - y1)
    #             return (x, y)
    #
    #     def pointOffset(xSrc, ySrc, yaw, offset):
    #         if xSrc == None or ySrc == None:
    #             return (None, None)
    #         else:
    #             x = xSrc + offset * math.sin(yaw)
    #             y = ySrc + offset * math.cos(yaw)
    #             return (x, y)
    #
    #     x = 0
    #     y = 0
    #
    #     pointsWithS = points
    #     (idx, ratio) = calculateRatio(pointsWithS, station)
    #     if (idx == None or ratio == None):
    #         return (None, None)
    #
    #     if idx == len(pointsWithS) - 1:
    #         (x, y) = pointOffset(pointsWithS[idx][0], pointsWithS[idx][1], pointsWithS[idx][2], offset)
    #         return (x, y)
    #
    #     else:
    #         (xTmp, yTmp) = linearInterpolation(pointsWithS[idx][0], pointsWithS[idx][1], pointsWithS[idx + 1][0],
    #                                            pointsWithS[idx + 1][1], ratio)
    #         (x, y) = pointOffset(xTmp, yTmp, pointsWithS[idx][2], offset)
    #         return (x, y)

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
    @classmethod #TODO 与主车相关的函数没有用到，忽略
    def myGetHostVehicleCoordinate(cls):
        return cls.externalInterface.getVehicleX(cls.g_nHostVehicleID), cls.externalInterface.getVehicleY(cls.g_nHostVehicleID)

    @classmethod #TODO 与主车相关的函数没有用到，忽略
    def myGetVehicleCoordinate(cls,vehID):
        return cls.externalInterface.getVehicleX(vehID), cls.externalInterface.getVehicleY(vehID)

    # 函数: myGetHostVehicleStationInLane
    # 用途: 获得主车在车道中的station
    # [o]: station
    @classmethod #TODO 与主车相关的函数没有用到，忽略
    def myGetHostVehicleStationInLane(cls):
        # return getDistanceFromLastJunction(g_nHostVehicleID)  # v9
        return cls.externalInterface.getDistanceFromLaneStart(cls.g_nHostVehicleID)  # v10

    # 函数: myGetHostVehicleSpeed
    # 用途: 获得主车的车速 [m/s]
    # [o]: speed
    @classmethod #TODO 与主车相关的函数没有用到，忽略
    def myGetHostVehicleSpeed(cls):
        return cls.externalInterface.getVehicleSpeed(cls.g_nHostVehicleID)

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

    # 函数: myCheckLaneExist
    # 用途: 判断车辆是否存在左车道或右车道
    # [i]: vehID - 车辆ID
    # [i]: dir -   -1：左侧；  1右侧；
    # [o]: Bool - True有车道，False无车道
    # @classmethod
    # def myCheckLaneExist(cls,vehID, dir):
    #     if (vehID < 0):
    #         return False
    #
    #     tmpID = 0
    #     if dir == 0:
    #         return False
    #     elif dir < 0:  # 左侧
    #         tmpID = cls.externalInterface.getLeftLeaderVehicle(vehID)
    #         log.info("GetLeftLeader".format(tmpID))
    #         if (tmpID >= 0) or (tmpID == -1):
    #             return True
    #         else:
    #             return False
    #     else:  # 右侧
    #         tmpID = cls.externalInterface.getRightLeaderVehicle(vehID)
    #         if (tmpID >= 0) or (tmpID == -1):
    #             return True
    #         else:
    #             return False

    @classmethod
    def myGetLeftLeaderVehicle(cls,vehID):
        target_vehicle = cls.externalInterface.getLeftLeaderVehicle(vehID)
        return target_vehicle

    @classmethod
    def myGetRightLeaderVehicle(cls,vehID):
        target_vehicle = cls.externalInterface.getRightLeaderVehicle(vehID)
        return target_vehicle

    @classmethod
    def myGetLeftFollowerVehicle(cls,vehID):
        target_vehicle = cls.externalInterface.getLeftFollowerVehicle(vehID)
        return target_vehicle

    @classmethod
    def myGetRightFollowerVehicle(cls,vehID):
        target_vehicle = cls.externalInterface.getRightFollowerVehicle(vehID)
        return target_vehicle

    @classmethod
    def myGetRoute(cls,vehID):
        return cls.externalInterface.getRoute(vehID)

    @classmethod
    def myGetVehicleList(cls):
        return cls.externalInterface.getVehicleList()

    @classmethod
    # API v10
    def myGetDistanceToLaneEnd(cls,vehID):
        return cls.externalInterface.getDistanceToLaneEnd(vehID)

    @classmethod
    def myGetDistanceFromLaneStart(cls,vehID):
        return cls.externalInterface.getDistanceFromLaneStart(vehID)

    @classmethod
    def myGetLaneID(cls,vehID):
        return cls.externalInterface.getVehicleLane(vehID)

    @classmethod
    def myGetLeftLaneID(cls,vehID):
        return cls.externalInterface.getLeftLane(cls.myGetLaneID(vehID))

    @classmethod
    def myGetLeftLaneIDBasedLane(cls,laneID):
        return cls.externalInterface.getLeftLane(laneID)

    @classmethod
    def myGetRightLaneID(cls,vehID):
        return cls.externalInterface.getRightLane(cls.myGetLaneID(vehID))

    @classmethod
    def myGetRightLaneIDBasedLane(cls,laneID):
        return cls.externalInterface.getRightLane(laneID)

    # @staticmethod
    # def myGetVehiclesInLane(laneID):
    #     return getLaneVehicles(laneID)

    @classmethod
    def myGetLaneShape(cls,laneID):
        return cls.externalInterface.getLaneShape(laneID)

    # @classmethod
    # def myGetPossibleDirections(cls,vehID):
    #     laneID = cls.myGetLaneID(vehID)
    #     return getValidDirections(laneID)

    @classmethod
    def myCheckInternalLane(cls,laneID):
        return cls.externalInterface.isInternalLane(laneID)

    @classmethod
    def myGetValidDirections(cls,laneID):
        return cls.externalInterface.getValidDirections(laneID)

    # @classmethod
    # def myGetNextLanes(cls,laneID, dir):
    #     return cls.externalInterface.getNextLanes(laneID, dir)

    @classmethod
    def myGetAllNextLanes(cls,laneID, dir):
        return cls.externalInterface.getAllNextLanes(laneID, dir)

    # @classmethod
    # def myGetToJunction(cls,laneID):
    #     return cls.externalInterface.getToJunction(laneID)

    @classmethod
    def myGetJunctionList(cls):
        return cls.externalInterface.getJunctionList()

    @classmethod
    def myGetIncomingLanes(cls,junctionId):
        return cls.externalInterface.getIncomingLanes(junctionId)

    @classmethod
    def myGetInternalLanes(cls,junctionId):
        return cls.externalInterface.getInternalLanes(junctionId)

    # @classmethod
    # def myGetLongitudinalDistance(cls,id1, id2):
    #     return cls.externalInterface.getLongitudinalDistance(id1, id2)
    #
    # @classmethod
    # def myGetLateralDistance(cls,id1, id2):
    #     return cls.externalInterface.getLateralDistance(id1, id2)

    @classmethod
    def myGetVehicleSpeed(cls,vehID):
        return cls.externalInterface.getVehicleSpeed(vehID)

    @classmethod
    def myGetVehicleX(cls,vehID):
        return cls.externalInterface.getVehicleX(vehID)

    @classmethod
    def myGetVehicleY(cls,vehID):
        return cls.externalInterface.getVehicleY(vehID)

    @classmethod
    def myGetVehicleYaw(cls,vehID):  # 返回弧度rad
        def myDeg2Rad(degree):
            return degree * math.pi / 180

        yaw = cls.externalInterface.getVehicleYaw(vehID)
        return myDeg2Rad(-yaw) + math.pi / 2  # 这里是从SUMO坐标系（正北，顺时针为正），转换为（正东，逆时针为正）

    @classmethod
    def myIsDeadEnd(cls,laneID):
        return cls.externalInterface.isDeadEnd(laneID)

    @classmethod
    def myMoveTo(cls,vehID, x, y, yaw,lane,s,l):
        curLaneID = cls.myGetLaneID(vehID)

        if cls.myIsDeadEnd(curLaneID):
            if (cls.myGetDistanceToLaneEnd(vehID) < 10):
                print('going to del ',vehID)
                cls.myDeleteVehicle(vehID)
                return

        #yaw = yaw * 180 / math.pi
        cls.externalInterface.moveTo(vehID, x, y, yaw,lane,s,l)

    # @classmethod
    # def my_Move_To(cls,vehID,x, y, yaw,lane,s,l,speed,acc):
    #     cls.externalInterface.moveTo(vehID,x, y, yaw,lane,s,l,speed,acc)

    # 函数: myCoorTransFromVehicleToSUMOWorld
    # 用途: 将车体坐标系下的轨迹，转换到世界坐标系下
    # [i] 车体坐标系下坐标 [ [x1,y1,yaw1,speed1,t1], ..., [xn,yn,yawn,speedn,tn] ]  注意，此处传入的yaw为rad；时间为s，以0开始；速度为m/s
    # [o] SUMO世界坐标系 [ [x1,y1,yaw1,speed1,t1], ..., [xn,yn,yawn,speedn,tn] ]  返回yaw为rad；时间为ms；速度为m/s
    @staticmethod
    def myCoorTransFromVehicleToSUMOWorld(curX, curY, curYaw, curTime, traj_in_vehicle):
        def myCoorTrans(old_x, old_y, new_in_old_x, new_in_old_y, new_from_old_theta):
            new_x = (old_x - new_in_old_x) * math.cos(new_from_old_theta) + (old_y - new_in_old_y) * math.sin(
                new_from_old_theta)
            new_y = (old_y - new_in_old_y) * math.cos(new_from_old_theta) - (old_x - new_in_old_x) * math.sin(
                new_from_old_theta)
            return (new_x, new_y)

        tran_x, trans_y = myCoorTrans(0, 0, curX, curY, curYaw)
        tran_yaw = -curYaw
        traj_in_world = []
        for e in traj_in_vehicle:
            (newX, newY) = myCoorTrans(e[0], e[1], tran_x, trans_y, tran_yaw)
            newYaw = math.pi / 2 - (e[2] + curYaw)  # e[2] + math.pi / 2
            newSpeed = e[3]
            newTime = e[4] * 1000 + curTime
            traj_in_world.append([newX, newY, newYaw, newSpeed, newTime])
        return traj_in_world

    # @classmethod
    # def mySetValidRoute(cls,vehID, printFlag=False):
    #     laneID = cls.myGetLaneID(vehID)
    #     if (laneID == ""): return False
    #     potentialDirs = cls.externalInterface.getValidDirections(laneID)
    #     dir = 0
    #     if (len(potentialDirs) > 0):
    #         dir = potentialDirs[int(cls.myGetRandomDouble(0, len(potentialDirs) - 1))]
    #     cls.externalInterface.changeRoute(vehID, cls.externalInterface.route_type(dir))
    #     if (printFlag == True):
    #         print("FromLane:", laneID, "Valid Dirs:", potentialDirs)
    #         print("SetRoute", dir)
    #     return True

    @staticmethod
    def myGetRandomDouble(min, max):
        rand = random.ranf()
        return min + rand * (max - min)

    """新增的封装函数"""

    @classmethod
    def myGetVehicleLane(cls,car_id):
        return cls.externalInterface.getVehicleLane(car_id)

    @classmethod
    def myGetVehicleAccel(cls,car_id):
        return cls.externalInterface.getVehicleAccel(car_id)

    @classmethod
    def myGetLeaderVehicle(cls,car_id):
        target_vehicle=cls.externalInterface.getLeaderVehicle(car_id)
        return target_vehicle

    @classmethod
    def myChangeSpeed(cls,car_id, current_speed, duration):
        cls.externalInterface.changeSpeed(car_id, current_speed)

    @classmethod
    def myGetVehicleRoute(cls,id):
        return cls.externalInterface.getRoute(id)

    @classmethod
    def myChangeRoute(cls,id, route):
        cls.externalInterface.changeRoute(id, route)

    @classmethod
    def myGetVehicleMaxSpeed(cls,car_id):
        return cls.externalInterface.getVehicleMaxSpeed(car_id)

    @classmethod
    def myDeleteVehicle(cls,car_id):
        cls.externalInterface.deleteVehicle(car_id)

    @classmethod
    def myGetInternalFoeLanes(cls,lane_id):
        return cls.externalInterface.getInternalFoeLanes(lane_id)  # 返回[(lane, type)]返回与该内部lane冲突的所有lane和冲突类型（Merging/MergingAdjacent/Crossing）

    @classmethod
    def myGetLaneVehicles(cls,lane_id):
        return cls.externalInterface.getLaneVehicles(lane_id)  # getLaneVehicles(lane):[id] 返回车道上所有车辆

    @classmethod
    def myGetVehicleLength(cls,car_id):
        return cls.externalInterface.getVehicleLength(car_id)

    @classmethod
    def myGetLaneLength(cls,lane_id):
        return cls.externalInterface.getLaneLength(lane_id)

    @classmethod
    def myGetVehicleLateralOffset(cls,car_id):
        return cls.externalInterface.getVehicleLateralOffset(car_id)

    @classmethod
    def myGetVehicleType(cls,car_id):
        return cls.externalInterface.getVehicleType(car_id)

    @classmethod
    def myGetVehicleMaxAccel(cls,car_id):
        return cls.externalInterface.getVehicleMaxAccel(car_id)

    @classmethod
    def myGetVehicleMaxDecel(cls,car_id):
        return cls.externalInterface.getVehicleMaxDecel(car_id)

    @staticmethod
    def myGetTrafficLightState(id, direction):
        return -1 #0代表没有灯

    @classmethod
    def myAddVehicle(cls,x, y, current_s, lane_id, speed=0, type=1):
        return cls.externalInterface.addVehicle(x, y, current_s,lane_id, speed ,type)


class myTimeMaster:
    def __init__(self):
        self.m_nLastTime = 0
        self.m_nCurrentTime = 0
        self.m_nDeltaTime = 0

    def updateTimeStamp(self, newTime):
        self.m_nLastTime = self.m_nCurrentTime
        self.m_nCurrentTime = newTime
        self.m_nDeltaTime = self.m_nCurrentTime - self.m_nLastTime
        return self.m_nDeltaTime

    def getDeltaTimeStep(self): return self.m_nDeltaTime

    def getCurrentTimeStamp(self): return self.m_nCurrentTime

    m_nLastTime = 0
    m_nCurrentTime = 0
    m_nDeltaTime = 0


