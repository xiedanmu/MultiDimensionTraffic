from Library.package_platformAPI.manager.LaneManager import LaneManager
from Library.package_platformAPI.manager.VehicleManager import VehicleManager
# from abc import ABCMeta, abstractmethod  # 实现接口的模块
class ExternalInterface():

    lane_manager = LaneManager
    vehicle_manager = VehicleManager

    @classmethod
    def getJunctionList(cls):
        pass

    @classmethod
    def getIncomingLanes(cls, junctionId):
        pass

    @classmethod
    def getLaneShape(cls,laneID):
        pass

    @classmethod
    def getInternalLanes(cls, junctionId):
        pass

    @classmethod
    def getValidDirections(cls,laneID):
        pass

    @classmethod
    def isInternalLane(cls,laneID):
        pass

    @classmethod
    def getVehicleSpeed(cls,vehicle_id):
        pass

    @classmethod
    def getVehicleX(cls,vehID):
        pass

    @classmethod
    def getVehicleY(cls,vehID):
        pass

    @classmethod
    def getVehicleLane(cls,vehID):
        pass

    @classmethod
    def getDistanceToLaneEnd(cls,vehID):
        pass

    @classmethod
    def getDistanceFromLaneStart(cls,vehID):
        pass

    @classmethod
    def getLeftLeaderVehicle(cls,vehID):
        pass

    @classmethod
    def getLeftLane(cls,laneID):
        pass

    @classmethod
    def getRightLeaderVehicle(cls,vehID):
        pass

    @classmethod
    def getRightLane(cls,laneID):
        pass

    @classmethod
    def getLeftFollowerVehicle(cls,vehID):
        pass

    @classmethod
    def getRightFollowerVehicle(cls,vehID):
        pass

    @classmethod
    def getVehicleList(cls):
        pass

    @classmethod
    def getVehicleYaw(cls,vehID):
        pass

    @classmethod
    def isDeadEnd(cls,laneID):
        pass

    @classmethod
    def getRoute(cls,vehID):
        pass

    @classmethod
    def getVehicleAccel(cls,vehID):
        pass

    @classmethod
    def getLeaderVehicle(cls,vehID):
        pass

    @classmethod
    def changeSpeed(cls,vehID, current_speed):
        pass

    @classmethod
    def changeRoute(cls,vehID,route):
        pass

    @classmethod
    def getVehicleMaxSpeed(cls,vehID):
        pass

    @classmethod
    def getLaneVehicles(cls,laneID):
        pass

    @classmethod
    def getVehicleLength(cls,vehID):
        pass

    @classmethod #TODO 这个在上层模块中已经计算出来了，不需要xml中的length
    def getLaneLength(cls,laneID):
        pass

    @classmethod
    def getVehicleMaxAccel(cls,vehID):
        pass

    @classmethod
    def getVehicleMaxDecel(cls,vehID):
        pass

    @classmethod
    def getVehicleLateralOffset(cls,vehID):
        pass

    @classmethod
    def getVehicleType(cls,vehID):
        pass

    @classmethod
    def getInternalFoeLanes(cls,lane_id):
        pass

    @classmethod
    def moveTo(cls,vehID,x, y, yaw,lane,s,l):
        pass

    @classmethod
    def addVehicle(cls,x, y, current_s,lane_id, speed ,type):
        pass

    @classmethod
    def deleteVehicle(cls,vehID):
        pass

    @classmethod
    def getAllNextLanes(cls,laneID, dir):
        pass