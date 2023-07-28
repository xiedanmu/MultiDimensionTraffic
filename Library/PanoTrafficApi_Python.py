###   多车干扰释放的交通模型接口.
###
###   v1.0  2022-08-13
###     初始版本
###   v1.1  2022-09-05
###     补充其他TraCI透传接口
###   v1.2  2022-09-15
###     添加接口
###         [moveVehicleByXY] 基于X,Y移动目标车辆
###         [getLanePoints] 获得车道点集
###         [getLaneFriction] 获得车道摩擦系数
###         [getLaneCurveCoeff] 获得前方曲率
###         [getLaneSlopeAngle] 获得前方坡度
###         [getScenarioVisibility] 获得场景可见度
###   v1.3  2022-09-22
###         [getCurTimeStamp] 获得当前仿真时间
###         [getAgentControlTimeInterval] 获得agent控制时间间隔
###         [getSimulationTimeStep] 获得仿真时间间隔
###   v1.4  2022-09-28
###         [getIncomingLanes] 获得驶入交叉口车道列表
###         [getInternalLanes] 获得交叉口内部车道列表
###         [getLaneLength] 获得车道长度
###   v1.5  2022-10-04
###     改为静态类导入dll中的函数接口，避免多次调用导致函数指针被释放
###   v1.6  2022-10-07
###     修改交叉口冲突车的返回类型，增加每个碰撞点是否被前车占用的符号位；并且所有未来碰撞点均会被输出。
###     添加接口
###         [CreateVehicleReleated] 基于相对位置创建车辆
###   v1.7  2022-10-10
###     修复byref函数使用错误，添加ctypes.
###     修复getVehicleXY返回值为c_double错误，使用c_double.value，返回python float类型
###   v1.8  2022-10-10
###     修复isLaneChange接口描述错误
###   v1.9  2022-10-11
###     添加SetRouteByDirections的next_junction_direction参数描述
###   v1.10 2022-11-01
###     修改冲突车信息接口及返回类型PanoTrafficAPI_GetFoesInfoInFutureCollisionPoints，为交通车添加抵达冲突点的”真实距离“
###     修改PanoTrafficAPI_SetVehicleRouteByDirections, PanoTrafficApi_GetNextLanes, PanoTrafficApi_GetValidDirections的接口定义，U_Turn = 3
###   v1.11 2022-11-04
###     添加接口
###         [CreateObjectXY] 基于X,Y坐标创建仿真物（支持非车类仿真物）
###         [MoveObjectByXYYaw] 基于X, Y, Yaw移动仿真物（支持非车类仿真物）
###     更新接口描述
###         [CreateVehicleReleated] 不支持非车类仿真物
###         [CreateVehicleInLaneSL] 支持非车类仿真物
###   v1.12 2022-11-24
###      添加接口
###         [GetLaneWidth] 获得车道宽度
###         [GetIsVehicleAllowed] 获得车道是否允许车辆行驶
###   v1.12.1 2022-12-17
###      修改接口名称，部分以前命名为Vehicle的接口，调整为Object，以区分可访问接口的物体类型
###         update 
###             [GetVehicleIsLaneChanging]
###             [SetObjectRouteByLaneList]
###             [CreateObjectInLaneSL]
###             [DeleteObject]
###             [MoveObjectBySpeed]
###             [SetObjectAgentControl]
###             [GetObjectAgentControl]
###             [GetObjectLane]
###             [GetObjectList]
###             [GetObjectYaw]
###             [GetObjectXY]
###             [GetObjectSpeed]
###             [GetObjectAccel]
###             [GetObjectTotalDistance]
###             [GetObjectType]
###             [GetObjectControlTimeInterval]
###
###             [CreateObjectByXY]  额外增加形参"LaneId"
###             [MoveObjectByXY]    额外增加形参"isPerformanceLimit"
###    v1.13 2023-03-30
###       废除接口PanoTrafficApi_SetObjectRouteByLaneList
###       修改接口，将PanoTrafficAPI_同一改为PanoTrafficApi_
###    v1.14 2023-04-19
###       修改接口MoveVehicleBySpeed参数
###       修改接口MoveVehicleByXY参数
###       修改接口SetObjectAgentControl含义
###       添加接口MoveVehicleBySpeedAcc/MoveObjectBySpeedAcc
###       添加接口MoveVehicleByLaneOffset/MoveObjectByLaneOffset
###       添加接口GetRoute
###       删除接口SetMultiDisturbanceFlag
###       删除接口GetObjectAgentControlTimeInterval
###    v1.15 2023-04-22
###       修改接口GetObjectControlTimeInterval含义
###    v1.15.1 2023-07-19
###       修复getLeftLaneId和getRightLaneId错误
###       修改getIsLaneChanging返回值定义
###       修改changeLane参数定义





import ctypes
# from ctypes import *
import os

#####
##### Interface structure declaration.
#####
MAX_FOE_VEHICLE_IN_COLLISIONPOINT = 3
MAX_COLLISIONPOINT_IN_ROUTE = 20
MAX_LANE_NAME_LENGTH = 128
MAX_VEH_COUNT = 400
MAX_VECTOR_SIZE = 200
MAX_LANE_POINT_COUNT = 2000
MAX_DIRECTION_SIZE = 5
MAX_NEXT_LANES_SIZE = 5
MAX_LANE_LIST_SIZE = 100
API_ReturnError = -1073741824

PanoTrafficAPI_LANE_NAME = ctypes.c_char * MAX_LANE_NAME_LENGTH
PanoTrafficAPI_VEH_LIST = ctypes.c_int * MAX_VEH_COUNT
PanoTrafficAPI_JUNCTION_LIST = PanoTrafficAPI_LANE_NAME * MAX_VECTOR_SIZE
PanoTrafficAPI_Direction_List = ctypes.c_int * MAX_DIRECTION_SIZE
PanoTrafficAPI_NEXTLANE_LIST = PanoTrafficAPI_LANE_NAME * MAX_NEXT_LANES_SIZE
PanoTrafficAPI_LANE_AXIS_POINT = ctypes.c_double * MAX_LANE_POINT_COUNT
PanoTrafficAPI_LANE_POINT_SET = 3 * PanoTrafficAPI_LANE_AXIS_POINT
PanoTrafficAPI_LANE_LIST = PanoTrafficAPI_LANE_NAME * MAX_LANE_LIST_SIZE



class PanoVehicleParameter:
    vehLength = 0
    vehWidth = 0
    vehMaxSpeed = 0
    vehMaxAccel = 0     # [0 ~ +Inf]
    vehMaxDecl = 0     # [-Inf ~ 0]


class PyLeaderInfo(ctypes.Structure):
    _fields_=[
        ("leaderId", ctypes.c_int),
        ("interval", ctypes.c_double),
        ("leaderType", ctypes.c_byte),
        ]

class PyFoeInfo(ctypes.Structure):
    _fields_=[
        ("foeId", ctypes.c_int),
        ("rangedInterval", ctypes.c_double),
        ("realInterval", ctypes.c_double),
    ]

class PyCollisionPointFoes(ctypes.Structure):
    _fields_=[
        ("vehCount", ctypes.c_int),
        ("foes", PyFoeInfo * MAX_FOE_VEHICLE_IN_COLLISIONPOINT),
    ]

class PyCollisionPointInfo(ctypes.Structure):
    _fields_=[
        ("collisionPointCount", ctypes.c_int),
        ("collisionPointInfo", PyCollisionPointFoes * MAX_COLLISIONPOINT_IN_ROUTE),
        ("egoRangedInterval", ctypes.c_double * MAX_COLLISIONPOINT_IN_ROUTE),
        ("egoRealInterval", ctypes.c_double * MAX_COLLISIONPOINT_IN_ROUTE),
        ("leaderOccupyFlag", ctypes.c_int * MAX_COLLISIONPOINT_IN_ROUTE),
    ]

class PyTrafficLightInfo(ctypes.Structure):
    _fields_=[
        ("trafficLightState", ctypes.c_int),
        ("nextChangeTimer", ctypes.c_double),
        ("distanceToLight", ctypes.c_double),
    ]

class PyVehicleParameter(ctypes.Structure):
    _fields_=[
        ("vehLength", ctypes.c_double),
        ("vehWidth", ctypes.c_double),
        ("vehMaxSpeed", ctypes.c_double),
        ("vehMaxAccel", ctypes.c_double),
        ("vehMaxDecl", ctypes.c_double),
    ]


#####
##### Interface.
#####

#  查询车辆是否正在换道
#  返回类型：
#       0：跟驰/保持；
#       1：正在左换道，且仍处于换道前的车道；
#       2：正在左换道，且已驶入目标车道；
#       -1：正在右换道，且仍处于换道前的车道；
#       -2：正在右换道，且已驶入目标车道；
def PanoTrafficApi_GetVehicleIsLaneChanging(vehId):
    return PanoTrafficInterface.func_GetVehicleIsLaneChanging(vehId)


#  查询车辆能维持安全的最大车速
#  返回类型：
#       最大车速
def PanoTrafficApi_GetNextSafetySpeed(vehId):
    return PanoTrafficInterface.func_GetNextSafetySpeed(vehId)



#  根据输入的Lane, s, l创建[物体]，并返回创建的vehId
#  注：可支持传入非车类仿真物类型。type和shape类型详见PanoSimDatabase中的描述
#  [type]:
#       Vehicle - Car = 0
#       Vehicle - Van = 1
#       Vehicle - Bus = 2
#       Vehicle - OtherVehicle = 3
#       Pedestrian = 10
#       NonMotorVehicle = 20
#       Others = 30
#  返回类型：
#       创建的[物体]ID，如果<0，代表创建失败
def PanoTrafficApi_CreateVehicleInLaneSL(laneId:str, s, l, speed, type:int, shape: int, driver: int):
    return PanoTrafficApi_CreateObjectInLaneSL(laneId, s, l, speed, type, shape, driver)
def PanoTrafficApi_CreateObjectInLaneSL(laneId:str, s, l, speed, type:int, shape: int, driver: int):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    newObjId = PanoTrafficInterface.func_CreateObjectInLaneSL(cstr_LaneId, s, l, speed, type, shape, driver)
    return newObjId


#  根据目标车辆，创建车辆，并返回创建的vehId
#  注：如果期望的station超过车道的正常长度，将返回错误值
#      不支持非车类仿真物。type和shape类型详见PanoSimDatabase中的描述
#  [type]:
#       Vehicle - Car = 0
#       Vehicle - Van = 1
#       Vehicle - Bus = 2
#       Vehicle - OtherVehicle = 3
#  [定义] targetVehId
#           目标参考车辆
#  [定义] laneDir
#           目标车道方向   -2：目标车右右车道； -1：目标车右车道； 0：当前车道； 1：目标车左车道； 2：目标车左左车道
#  [定义] deltaS
#           间距，[-inf ~ inf]
#  [定义] laneOffset
#           相对于车道中心线的距离，左侧为正
#  返回类型：
#       创建的车辆ID，如果<0，代表创建失败
def PanoTrafficApi_CreateVehicleRelated(targetVehId:int, laneDir:int, deltaS, laneOffset, speed, type:int, shape: int, driver: int):
    newVehId = PanoTrafficInterface.func_CreateVehicleRelated(targetVehId, laneDir, deltaS, laneOffset, speed, type, shape, driver)
    return newVehId


#  根据输入的x, y创建物体，并返回创建的objId
#  注：可支持传入非车类仿真物类型。type和shape类型详见PanoSimDatabase中的描述
#  [type]:
#       Vehicle - Car = 0
#       Vehicle - Van = 1
#       Vehicle - Bus = 2
#       Vehicle - OtherVehicle = 3
#       Pedestrian = 10
#       NonMotorVehicle = 20
#       Others = 30
#  返回类型：
#       创建的仿真物ID，如果<0，代表创建失败
def PanoTrafficApi_CreateVehicleXY(laneId:str, targetX, targetY, speed, type:int, shape:int, driver:int):
    return PanoTrafficApi_CreateObjectXY(laneId, targetX, targetY, speed, type, shape, driver)
def PanoTrafficApi_CreateObjectXY(laneId:str, targetX, targetY, speed, type:int, shape:int, driver:int):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    newObjId = PanoTrafficInterface.func_CreateObjectXY(cstr_LaneId, targetX, targetY, speed, type, shape, driver)
    return newObjId


#  根据传入的"期望速度"移动[物体]，如果limit参数
#  [定义] duration
#            多长时间后达到期望车速
#  返回类型：
#       如果<0，代表移动失败
def PanoTrafficApi_MoveVehicleBySpeed(objId, expSpeed, duration):
    return PanoTrafficApi_MoveObjectBySpeed(objId, expSpeed, duration)
def PanoTrafficApi_MoveObjectBySpeed(objId, expSpeed, duration):
    return PanoTrafficInterface.func_MoveObjectBySpeed(objId, expSpeed, duration)

#  根据传入的"最终速度"和"期望加速度"移动[物体]
#  [定义] finalSpeed
#            最终要达到的车速
#  [定义] expAcc
#            多长时间内达到期望车速
#            参数缺省为API_ReturnError，代表尽快达到期望车速
#  返回类型：
#       如果<0，代表移动失败
def PanoTrafficApi_MoveVehicleBySpeedAcc(objId, finalSpeed, expAcc = API_ReturnError):
    return PanoTrafficApi_MoveObjectBySpeedAcc(objId, finalSpeed, expAcc)
def PanoTrafficApi_MoveObjectBySpeedAcc(objId, finalSpeed, expAcc = API_ReturnError):
    return PanoTrafficInterface.func_MoveObjectBySpeedAcc(objId, finalSpeed, expAcc)


#  根据传入的“偏移”移动[物体]
#  [定义] expOffset
#             期望的基于当前车道中心线的偏移 [m]
#  [定义] duration
#             希望到达期望偏移的时间 [s]
def PanoTrafficApi_MoveVehicleByLaneOffset(objId, expOffset, duration = 2.0):
    return PanoTrafficApi_MoveObjectByLaneOffset(objId, expOffset, duration)
def PanoTrafficApi_MoveObjectByLaneOffset(objId, expOffset, duration = 2.0):
    return PanoTrafficInterface.func_MoveObjectByLaneOffset(objId, expOffset, duration)

#  根据传入的"期望目标点坐标"移动[物体]
#  返回类型：
#       如果<0，代表移动失败
def PanoTrafficApi_MoveObjectByXY(vehId, expX, expY, expSpeed):
    return PanoTrafficInterface.func_MoveObjectByXY(vehId, expX, expY, expSpeed)

#  根据传入的"期望目标点坐标"和“朝向”移动仿真物
#  [定义] expYaw
#             单位rad。右手坐标系，X轴正方向为0，逆时针为正。
#  返回类型：
#       如果<0，代表移动失败
def PanoTrafficApi_MoveObjectByXYYaw(vehId, expX, expY, expYaw, expSpeed):
    return PanoTrafficInterface.func_MoveObjectByXYYaw(vehId, expX, expY, expYaw, expSpeed)



#  [车辆]换道请求
#  [定义] lane_change_dir
#             1 : left;
#            -1 : right;
#             0 ： 如果此时车辆正在换道，则取消换道并保持当前offset行驶
#  返回类型：
#       如果<0，代表请求失败
def PanoTrafficApi_ChangeLane(vehId, lane_change_dir, duration = 2.0):
    ret = PanoTrafficInterface.func_ChangeLane(vehId, lane_change_dir, duration)
    return ret



#  返回vehId未来的多个碰撞点的冲突车信息
#  返回类型：
#       ( 前车ID, 本车与前车距离, 索引前车的类型 )
#  [定义] 本车与前车距离
#            前车车尾与本车车头的距离，正常情况下，该值应>0
#  [定义] 索引前车的类型
#            0 - 前车与本车在相同的车道
#            1 - 前车在本车的未来车道上
#            2 - 前车在本车的“同出发点”碰撞点的冲突车道上
def PanoTrafficApi_GetLeader(vehId):
    leaderInfo = PanoTrafficInterface.func_GetRealLeader(vehId)
    return (leaderInfo.leaderId, leaderInfo.interval, leaderInfo.leaderType)



#  返回vehId未来的多个碰撞点的冲突车信息
#  返回类型：
#       [ (本车距离该碰撞点的距离（距离控制区域的距离）,
#          本车距离该碰撞点的真实距离（距离冲突点的距离）,
#             [(冲突车1-ID, 冲突车1距离该碰撞点的距离, 冲突1距离该碰撞点的真实距离),
#              (冲突车2-ID, 冲突车2距离该碰撞点的距离, 冲突2距离该碰撞点的真实距离),
#              ....
#             ],
#         该冲突点是否被前车占用
#         ),
#         ....
#       ]
#   [定义] 本车距离该碰撞点的距离：
#               车辆距离冲突点'控制区域'的距离；
#               当车头在碰撞点以前，返回正数距离；
#               当车尾在碰撞点控制范围，返回0；
#               一般不存在车尾超出控制范围的情况，该情况碰撞点会删除该车辆信息，即不再占有该碰撞点；
#               [  point     ]______ <- Vehicle
#
#   [定义] 本车距离该碰撞点的真实距离：
#               车辆距离冲突'点'的距离；
#               [  point_____]______ <- Vehicle
#   [maxVehicleCountInFoe] 交叉口冲突点的冲突车，可以返回的最大车辆数目：
#                              默认为3，可以传入1.
#
def PanoTrafficApi_GetFoesInfoInFutureCollisionPoints(vehId, maxVehicleCountInFoe = MAX_FOE_VEHICLE_IN_COLLISIONPOINT):
    foesInPointsInfo = PanoTrafficInterface.func_GetFoeInfo(vehId)
    maxVehicleCountInFoe = min(maxVehicleCountInFoe, MAX_FOE_VEHICLE_IN_COLLISIONPOINT)
   
    lCollisionPointsInfo = []
    for pointIdx in range(min(foesInPointsInfo.collisionPointCount, MAX_COLLISIONPOINT_IN_ROUTE)):
        egoRealDis = foesInPointsInfo.egoRealInterval[pointIdx]
        egoRangedDis = foesInPointsInfo.egoRangedInterval[pointIdx]
        foesInfo = foesInPointsInfo.collisionPointInfo[pointIdx]
        leaderOccupy = foesInPointsInfo.leaderOccupyFlag[pointIdx]

        lFoes = []
        for vehIdx in range(min(maxVehicleCountInFoe, foesInfo.vehCount)):
            foeInfo = foesInfo.foes[vehIdx]
            lFoes.append( (foeInfo.foeId, foeInfo.rangedInterval, foeInfo.realInterval) )

        lCollisionPointsInfo.append( (egoRangedDis, egoRealDis, lFoes, leaderOccupy) )
    return lCollisionPointsInfo


#  返回vehId在下一个路口的交通灯信息
#  返回类型：
#       (交通灯颜色, 交通灯剩余时间, 距离交通灯的距离)
#   [定义] 交通灯颜色：
#               -1 - empty or error.
#               0  - red
#               1  - yellow
#               2  - green
#   [dir] 可以传入非默认路径的行驶方向。
#               默认为-1073741824，即返回默认路径（行驶方向）的交通灯信息；
#               非默认时，则会根据传入的行驶方向，返回交通灯信息；
#
def PanoTrafficApi_GetTrafficLightInfo(vehId, dir = -1073741824):
    tl = PanoTrafficInterface.func_GetTrafficLightInfo(vehId, dir)
    return (tl.trafficLightState, tl.nextChangeTimer, tl.distanceToLight)


#  废除.
#  为[物体]添加行驶路径，传入所有车道（包括internal lane和normal lane），且[物体]所在当前车道必须是[normal lane]
#  返回类型：
#       如果<0，代表设置失败
def PanoTrafficApi_SetObjectRouteByLaneList(objId, laneList : list):
    laneCount = len(laneList)
    cLaneListType = PanoTrafficAPI_LANE_NAME * laneCount
    cLaneList = cLaneListType()
    for i in range(len(laneList)):
        iterLane = laneList[i]
        for j in range(len(iterLane)):
            cLaneList[i][j] = ctypes.c_char(iterLane[j].encode('utf-8'))
    ret = PanoTrafficInterface.func_SetObjectRouteByLaneList(objId, laneCount, cLaneList)
    return ret


#  为车辆添加行驶路径，传入未来各个交叉口的行驶方向 directions: list[next_junction_direction]
#  [定义] next_junction_direction
#               0: straight;
#               1: left;
#               2: right;
#               3: u_turn;
#               4: unknown;
#  返回类型：
#       如果<0，代表设置失败
def PanoTrafficApi_SetVehicleRouteByDirections(vehId, curLaneId, directions : list):
    cCurLaneId = ctypes.c_char_p(curLaneId.encode('utf-8'))
    cDirectionCount = len(directions)
    cDirs = (ctypes.c_int * cDirectionCount)(*directions)
    ret = PanoTrafficInterface.func_SetVehicleRouteByDirections(vehId, cCurLaneId, cDirectionCount, cDirs)
    return ret


# #  根据传入的"加速度"移动车辆，如果limit参数
# #  [定义] expOffset
# #            默认0.0，即车辆始终行驶在车道中心线上
# #  [定义] vehPerformanceLimit
# #            默认为1，当参数>0时，API会根据默认的车辆性能MaxAcc/MaxDec/MaxSpeed/MaxLateralSpeed，约束车辆行为
# #            如果传入0，则API不处理，直接使用acceleration，并限制最低车速为0（即不会倒车）
# #  返回类型：
# #       如果<0，代表移动失败
# def PanoTrafficApi_MoveVehicleByAcceleration(vehId, acceleration, expOffset = 0.0, vehPerformanceLimit = 1):
#     return PanoTrafficInterface.func_MoveVehicleByAcceleration(vehId, acceleration, expOffset, vehPerformanceLimit)



#  传入设置参数  当距离小于DistanceToArrival 或 到达时间小于TimeToArrival时，开始计算碰撞点信息。
#              如果TimeToArrival<0，则不考虑车速影响，仅使用距离做判断
#  返回类型：
#       如果<0，代表移动失败
def PanoTrafficApi_SetJunctionParameters(ControlRight_DistanceToArrival, ControlRight_TimeToArrival):
    return PanoTrafficInterface.func_SetJunctionParameters(ControlRight_DistanceToArrival, ControlRight_TimeToArrival)


#  请求在特定车道中设置一个恒定车辆数目的交通流
#  [定义] departureSpeed
#            最大发车车速，API会根据前车距离，自动限定一个安全发车车速
#  [定义] departureStation
#            在车道中的发车位置
#  返回类型：
#       如果<0，代表移动失败
def PanoTrafficApi_SetConstantTrafficFlowInLane(laneId, vehCount : int, departureSpeed, departureStation = 0.0):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    ret = PanoTrafficInterface.func_SetConstantTrafficFlowInLane(cstr_LaneId, vehCount, departureSpeed, departureStation)
    return ret


#  设置某一物体的控制源，（最大）每隔多长时间/距离，调用该交通物的InstanceUpdate脚本，
#  亦可以传入如控制时间、控制距离等参数，当达到条件后，优先调用InstanceUpdate脚本
#  当单例交通物运行时间足够步长调用时，仍然会每个步长都调用InstanceUpdate.
#  [定义] state
#           >0： 激活外部控制源    0: 激活API内部控制源
#  [定义] checkTerminate
#           0: 每隔步长都调用   1: 每隔[时间]调用   2: 每隔[行驶距离]调用
#  [定义] condition
#           传入的“条件”（阈值），当超过该阈值时，优先调用InstanceUpdate脚本
#           时间 [Unit: s]，距离 [Unit: m]
#  返回类型：
#       如果<0，代表设置失败
def PanoTrafficApi_SetObjectAgentControl(objId: int, state: int, checkTerminate = 0, condition = 0.01):
    return PanoTrafficInterface.func_SetObjectAgentControl(objId, state, checkTerminate, condition)


#  获得某一物体的控制源，如果是正在被agent控制，返回1
#  返回类型：
#       如果<0，代表获取失败；
#       如果=0，正在API内部控制；
#       如果>0，正在被agent控制；
def PanoTrafficApi_GetObjectAgentControl(objId: int):
    return PanoTrafficInterface.func_GetObjectAgentControl(objId)





#  删除交通[物体]
#  [定义] id
#  返回类型：
#       如果<0，代表设置失败
def PanoTrafficApi_DeleteObject(id : int):
    return PanoTrafficInterface.func_DeleteObject(id)



#  设置交叉口处的行驶路径
#  [定义] id
#         direction
#           "0" - straight;
#           "1" - left-turn;
#           "2" - right-turn;
#           "3" - u-turn;
#           "4" - unkown;
#  返回类型：
#       如果<0，代表设置失败
def PanoTrafficApi_ChangeRoute(id : int, direction : int):
    return PanoTrafficInterface.func_ChangeRoute(id, direction)

#  获得交叉口处的行驶路径
#  [定义] id
#  返回类型：
#         direction
#           "-1"- no object
#           "0" - straight;
#           "1" - left-turn;
#           "2" - right-turn;
#           "3" - u-turn;
#           "4" - unkown;
def PanoTrafficApi_GetRoute(id : int):
    return PanoTrafficInterface.func_GetRoute(id)



#  获得[物体]所在车道ID
#  [定义] id
#  返回类型：
#       车道ID，如果是""，代表访问失败
def PanoTrafficApi_GetVehicleLane(id : int):
    return PanoTrafficApi_GetObjectLane(id)
def PanoTrafficApi_GetObjectLane(id : int):
    cLaneId = PanoTrafficAPI_LANE_NAME()
    ret = PanoTrafficInterface.func_GetObjectLane(id, cLaneId)
    if ret < 0:
        return ""
    else:
        return str(cLaneId.value, 'utf-8')


#  获得场景内的所有[物体]ID
#  [定义] ignoreHost
#           0: 包含主车；   1: 排除主车
#  返回类型：
#       [物体]ID列表[]
def PanoTrafficApi_GetVehicleList(ignoreHost=0):
    return PanoTrafficApi_GetObjectList(ignoreHost)
def PanoTrafficApi_GetObjectList(ignoreHost=0):
    pycVehCount = ctypes.c_int(0)
    pycVehList = PanoTrafficAPI_VEH_LIST()
    ret = PanoTrafficInterface.func_GetObjectList(ignoreHost, ctypes.byref(pycVehCount), pycVehList)

    vehCount = min(pycVehCount.value, MAX_VEH_COUNT)
    vehList = []
    for i in range(vehCount):
        vehList.append(pycVehList[i])

    return vehList


#  获得车道内的所有车辆ID
#  [定义] laneId
#           车道ID
#  返回类型：
#       车道内的车辆ID列表[]
def PanoTrafficApi_GetVehicleListInLane(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    pycVehCount = ctypes.c_int(0)
    pycVehList = PanoTrafficAPI_VEH_LIST()
    ret = PanoTrafficInterface.func_GetVehicleListInLane(cstr_LaneId, ctypes.byref(pycVehCount), pycVehList)

    vehCount = min(pycVehCount.value, MAX_VEH_COUNT)
    vehList = []
    for i in range(vehCount):
        vehList.append(pycVehList[i])

    return vehList


#  获得车道内一定范围的的所有车辆ID
#  [定义] laneId:
#           车道ID
#         startStation/endStation:
#           起始/终止距离(Station)
#  返回类型：
#       车道距离内的车辆ID列表[]
def PanoTrafficApi_GetVehicleListInLaneWithinRange(laneId : str, startStation, endStation):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    pycVehCount = ctypes.c_int(0)
    pycVehList = PanoTrafficAPI_VEH_LIST()
    ret = PanoTrafficInterface.func_GetVehicleListInLaneWithinRange(cstr_LaneId, startStation, endStation, ctypes.byref(pycVehCount), pycVehList)

    vehCount = min(pycVehCount.value, MAX_VEH_COUNT)
    vehList = []
    for i in range(vehCount):
        vehList.append(pycVehList[i])

    return vehList


#  获得车辆在车道的station
#  [定义] vehId:
#           车辆ID
#  返回类型：
#       station
def PanoTrafficApi_GetDistanceFromLaneStart(vehId: int):
    return PanoTrafficInterface.func_GetDistanceFromLaneStart(vehId)


#  获得车辆离车道终点的距离
#  [定义] vehId:
#           车辆ID
#  返回类型：
#       distance
def PanoTrafficApi_GetDistanceToLaneEnd(vehId: int):
    return PanoTrafficInterface.func_GetDistanceToLaneEnd(vehId)


#  获得车辆与车道中心线的偏移量
#  [定义] vehId:
#           车辆ID
#  返回类型：
#       > 0: 车辆在车道中心线的左侧；
#       < 0: 车辆在车道中心线的右侧；
#       = API_ReturnError: 访问错误
def PanoTrafficApi_GetLateralOffset(vehId: int):
    return PanoTrafficInterface.func_GetLateralOffset(vehId)


#  获得车辆所在/即将驶入的交叉口ID
#  [定义] vehId:
#           车辆ID
#  返回类型：
#       (-1, ""  ): 不在交叉口内，或访问出错
#       (0,  "XX"): 即将驶入的交叉口ID
#       (1,  "XX"): 处于的交叉口ID
def PanoTrafficApi_GetVehicleJunction(vehId: int):
    cJunctionId = PanoTrafficAPI_LANE_NAME()

    ret = PanoTrafficInterface.func_GetVehicleJunction(vehId, cJunctionId)
    return (ret, str(cJunctionId.value, 'utf-8'))


#  获得物体朝向 （坐标系：X正为0，逆时针为+）
#  [定义] objId:
#           [物体]ID
#  返回类型：
#       yaw [rad]
def PanoTrafficApi_GetVehicleYaw(objId: int):
    return PanoTrafficApi_GetObjectYaw(objId)
def PanoTrafficApi_GetObjectYaw(objId: int):
    return PanoTrafficInterface.func_GetObjectYaw(objId)


#  获得XY坐标
#  [定义] objId:
#           [物体]ID
#  返回类型：
#       (x, y)
#           API_ReturnError: Error
def PanoTrafficApi_GetVehicleXY(objId: int):
    return PanoTrafficApi_GetObjectXY(objId)
def PanoTrafficApi_GetObjectXY(objId: int):
    cX = ctypes.c_double(0)
    cY = ctypes.c_double(0)
    ret = PanoTrafficInterface.func_GetObjectXY(objId, ctypes.byref(cX), ctypes.byref(cY))
    if ret < 0:
        return (API_ReturnError, API_ReturnError)
    else:
        return (cX.value, cY.value)


#  获得[物体]车速
#  [定义] objId:
#           [物体]ID
#  返回类型：
#       speed [m/s]
#           API_ReturnError: Error
def PanoTrafficApi_GetVehicleSpeed(objId: int):
    return PanoTrafficApi_GetObjectSpeed(objId)
def PanoTrafficApi_GetObjectSpeed(objId: int):
    return PanoTrafficInterface.func_GetObjectSpeed(objId)


#  获得[物体]加速度
#  [定义] vehId:
#           [物体]ID
#  返回类型：
#       Ax [m/s2]
#           API_ReturnError: Error
def PanoTrafficApi_GetVehicleAccel(objId: int):
    return PanoTrafficApi_GetObjectAccel(objId)
def PanoTrafficApi_GetObjectAccel(objId: int):
    return PanoTrafficInterface.func_GetObjectAccel(objId)


#  获得[物体]从创建开始，行驶的里程总计
#  [定义] vehId:
#           [物体]ID
#  返回类型：
#       distance [m]
#           <0: Error
def PanoTrafficApi_GetVehicleTotalDistance(objId: int):
    return PanoTrafficApi_GetObjectTotalDistance(objId)
def PanoTrafficApi_GetObjectTotalDistance(objId: int):
    return PanoTrafficInterface.func_GetObjectTotalDistance(objId)


#  获得车辆周边交通车信息
#  [定义] vehId:
#           车辆ID
#  返回类型：
#       vehId [m]
#           API_ReturnError: 无车道;
#           -1: 无车;
#           >=0: 有车信息;
#   NOTE: 该接口仅能访问本车道内100m距离内的前车信息，也可以使用GetLeader获得额外信息
def PanoTrafficApi_GetLeaderVehicleInLane(vehId: int):
    return PanoTrafficInterface.func_GetLeaderVehicleInLane(vehId)

def PanoTrafficApi_GetLeftLeaderVehicle(vehId: int):
    return PanoTrafficInterface.func_GetLeftLeaderVehicle(vehId)

def PanoTrafficApi_GetRightLeaderVehicle(vehId: int):
    return PanoTrafficInterface.func_GetRightLeaderVehicle(vehId)

def PanoTrafficApi_GetLeftFollowerVehicle(vehId: int):
    return PanoTrafficInterface.func_GetLeftFollowerVehicle(vehId)

def PanoTrafficApi_GetRightFollowerVehicle(vehId: int):
    return PanoTrafficInterface.func_GetRightFollowerVehicle(vehId)

def PanoTrafficApi_GetFollowerVehicle(vehId: int):
    return PanoTrafficInterface.func_GetFollowerVehicle(vehId)


#  获得两车之间的纵向距离 Dis = ID1 - ID2
#  [定义] vehId1 / vehId2:
#           车辆ID
#  返回类型：
#       distance [m]
#           >0: ID1在ID2前方
#           <0: ID1在ID2后方
#           API_ReturnError: Error
def PanoTrafficApi_GetLongitudinalDistance(vehId1, vehId2):
    return PanoTrafficInterface.func_GetLongitudinalDistance(vehId1, vehId2)


#  获得两车之间的侧向距离 Dis = ID1 - ID2
#  [定义] vehId1 / vehId2:
#           车辆ID
#  返回类型：
#       distance [m]
#           >0: ID1在ID2左侧
#           <0: ID1在ID2右侧
#           API_ReturnError: Error
def PanoTrafficApi_GetLateralDistance(vehId1, vehId2):
    return PanoTrafficInterface.func_GetLateralDistance(vehId1, vehId2)


#  判断当前车道是否具备左侧车道
#  [定义] laneId
#           当前车道
#  返回类型：
#           False: 无左车道
#           True: 有左车道
def PanoTrafficApi_IsLeftLaneExist(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    ret = PanoTrafficInterface.func_IsLeftLaneExist(cstr_LaneId)
    if ret > 0:
        return True
    else:
        return False


#  判断当前车道是否具备右侧车道
#  [定义] laneId
#           当前车道
#  返回类型：
#           False: 无右车道
#           True: 有右车道
def PanoTrafficApi_IsRightLaneExist(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    ret = PanoTrafficInterface.func_IsRightLaneExist(cstr_LaneId)
    if ret > 0:
        return True
    else:
        return False


#  获得当前车道的左侧车道ID
#  [定义] laneId
#           当前车道
#  返回类型：
#           "": 无左车道
#           其他: 左侧车道ID
def PanoTrafficApi_GetLeftLaneId(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    cstr_LeftLaneId = PanoTrafficAPI_LANE_NAME()

    ret = PanoTrafficInterface.func_GetLeftLaneId(cstr_LaneId, cstr_LeftLaneId)
    if ret < 0:
        return ""
    else:
        return str(cstr_LeftLaneId.value, 'utf-8')


#  获得当前车道的右侧车道ID
#  [定义] laneId
#           当前车道
#  返回类型：
#           "": 无右车道
#           其他: 右侧车道ID
def PanoTrafficApi_GetRightLaneId(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    cstr_RightLaneId = PanoTrafficAPI_LANE_NAME()

    ret = PanoTrafficInterface.func_GetRightLaneId(cstr_LaneId, cstr_RightLaneId)
    if ret < 0:
        return ""
    else:
        return str(cstr_RightLaneId.value, 'utf-8')


##check
#  获得场景内所有JunctionId
#  返回类型：
#           []: 所有JunctionID
def PanoTrafficApi_GetJunctionList():
    cJunctionList = PanoTrafficAPI_JUNCTION_LIST()

    ret = PanoTrafficInterface.func_GetJunctionList(cJunctionList)
    if ret <= 0:
        return []
    else:
        junctionList = []
        for i in range(ret):
            junctionList.append(str(cJunctionList[i].value, 'utf-8'))
        return junctionList


#  获得道路可行驶的方向
#  [定义] laneId
#           当前车道
#  返回类型：
#           [dir1, ...]: 所有可行驶的方向
#               0: straight;
#               1: left;
#               2: right;
#               3: u_turn;
#               4: unknown;
def PanoTrafficApi_GetValidDirections(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    carray_directions = PanoTrafficAPI_Direction_List()

    ret = PanoTrafficInterface.func_GetValidDirections(cstr_LaneId, carray_directions)
    if ret <= 0:
        return []
    else:
        directions = []
        for i in range(ret):
            directions.append(carray_directions[i])
        return directions


##check
#  基于行驶方向，获得下一段路径（车道）
#  [定义] laneId
#           当前车道
#         direction
#               0: straight;
#               1: left;
#               2: right;
#               3: u_turn;
#               4: unknown;
#  返回类型：
#           [nextLane1, nextLane2, ...]: 下一个车道
def PanoTrafficApi_GetNextLanes(laneId : str, direction : int):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    carray_nextLaneIds = PanoTrafficAPI_NEXTLANE_LIST()

    ret = PanoTrafficInterface.func_GetNextLanes(cstr_LaneId, direction, carray_nextLaneIds)
    if ret <= 0:
        return []
    else:
        nextLanes = []
        for i in range(ret):
            nextLanes.append(str(carray_nextLaneIds[i].value, 'utf-8'))
        return nextLanes


#  获得当前车道驶向的JunctionID
#  [定义] laneId
#           当前车道
#  返回类型
#           "": error
#           其他: toJunctionId
def PanoTrafficApi_GetLaneToJunction(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    cstr_ToJunctionId = PanoTrafficAPI_LANE_NAME()

    ret = PanoTrafficInterface.func_GetLaneToJunction(cstr_LaneId, cstr_ToJunctionId)
    if ret < 0:
        return ""
    return str(cstr_ToJunctionId.value, 'utf-8')


#  获得目标车道的点集 [(X,Y,Z), ... ]
#  [定义] laneId
#           目标车道
#  返回类型
#           [(X,Y,Z), ... ]
def PanoTrafficApi_GetLanePoints(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    cpp_Points = PanoTrafficAPI_LANE_POINT_SET()

    pointSize = PanoTrafficInterface.func_GetLanePoints(cstr_LaneId, cpp_Points)
    pyLanePoint = []
    for i in range(pointSize):
        pyLanePoint.append( (cpp_Points[0][i], cpp_Points[1][i], cpp_Points[2][i]) )
    return pyLanePoint


#  获得车道的摩擦系数
#  [定义] laneId
#           目标车道
#  返回类型
#           double 摩擦系数
def PanoTrafficApi_GetLaneFrictionCoeff(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    return PanoTrafficInterface.func_GetLaneFrictionCoeff(cstr_LaneId)

#  基于车辆当前位置，获得前方预瞄点的曲率（支持迭代向前查找）
#  [定义] vehId
#           目标车辆
#        preViewStation
#           预瞄距离
#  返回类型
#           double 曲率   >0:左曲   <0:右曲   =0:直线
def PanoTrafficApi_GetLaneCurveCoeff(vehId : int, preViewStation):
    return PanoTrafficInterface.func_GetLaneCurveCoeff(vehId, preViewStation)

#  基于车辆当前位置，获得前方预瞄点的坡度（支持迭代向前查找）
#  返回值为 tan(theta)
#  [定义] vehId
#           目标车辆
#        preViewStation
#           预瞄距离
#  返回类型
#           double 坡度   unit:[]
def PanoTrafficApi_GetLaneSlopeAngle(vehId : int, preViewStation):
    return PanoTrafficInterface.func_GetLaneSlopeAngle(vehId, preViewStation)


#  获得场景可见度参数
#  返回类型
#           double 可见度   unit:[m]
def PanoTrafficApi_GetScenarioVisibility():
    return PanoTrafficInterface.func_GetScenarioVisibility()


#  获得当前车道驶出的JunctionID
#  [定义] laneId
#           当前车道
#  返回类型
#           "": error
#           其他: fromJunctionId
def PanoTrafficApi_GetLaneFromJunction(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    cstr_FromJunctionId = PanoTrafficAPI_LANE_NAME()

    ret = PanoTrafficInterface.func_GetLaneFromJunction(cstr_LaneId, cstr_FromJunctionId)
    if ret < 0:
        return ""
    return str(cstr_FromJunctionId.value, 'utf-8')


#  检查当前道路是不是断路，即必须在终点前产生换道
#  [定义] laneId
#           当前车道
#  返回类型
#           False: 不是断路，可继续行驶
#           True: 断路，必须提前换道
def PanoTrafficApi_GetIsDeadEnd(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    ret = PanoTrafficInterface.func_GetIsDeadEnd(cstr_LaneId)
    if ret == 0:
        return False
    else:
        return True


#  检查当前道路是不是内部车道，即是不是交叉口中的车道
#  [定义] laneId
#           当前车道
#  返回类型
#           False: 不是内部车道
#           True: 是内部车道，交叉口内车道
def PanoTrafficApi_GetIsInternalLane(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    ret = PanoTrafficInterface.func_GetIsInternalLane(cstr_LaneId)
    if ret == 0:
        return False
    else:
        return True


#  获得[物体]类型
#  [定义] objId
#           [物体]ID
#  返回类型
#           Car = 0,
#           Van = 1,
#           Bus = 2,
#           OtherVehicle = 3,
#           Pedestrian = 10,
#           NonMotorVehicle = 20,
#           Others = 30
def PanoTrafficApi_GetObjectType(objId : int):
    return PanoTrafficInterface.func_GetObjectType(objId)


#  获得驾驶员类型
#  [定义] vehId
#           车辆ID
#  返回类型
#           normal = 0,/// @brief 普通
#           cautious = 1,/// @brief 谨慎
#           radicalization = 2/// @brief 激进
def PanoTrafficApi_GetDriverType(vehId : int):
    return PanoTrafficInterface.func_GetDriverType(vehId)


#  获得车辆参数
#  [定义] vehId
#           车辆ID
#  返回类型
#         PanoVehicleParameter
def PanoTrafficApi_GetVehicleParameter(vehId : int):
    cVehParamter = PanoTrafficInterface.func_GetVehicleParameter(vehId)
    ret = PanoVehicleParameter()
    ret.vehLength = cVehParamter.vehLength
    ret.vehWidth = cVehParamter.vehWidth
    ret.vehMaxSpeed = cVehParamter.vehMaxSpeed
    ret.vehMaxAccel = cVehParamter.vehMaxAccel
    ret.vehMaxDecl = cVehParamter.vehMaxDecl

    return ret


#  获得仿真的时间信息
#  返回类型
#         (当前时间戳，仿真步长）  单位：s
def PanoTrafficApi_GetSimulationTimeInfo():
    curTime = PanoTrafficInterface.func_GetSimulationCurTimeStamp()
    deltaTime = PanoTrafficInterface.func_GetSimulationTimeStep()
    return (curTime, deltaTime)


#  获得距上一次Python模型控制的仿真时间间隔
#  [定义] objId
#           [物体]ID
#  返回类型
#          double  时间间隔curTime - lastAgentControlTime  单位: s
def PanoTrafficApi_GetObjectControlTimeInterval(objId : int):
    return PanoTrafficInterface.func_GetObjectControlTimeInterval(objId)


#  获得交叉口驶入车道列表
#  [定义] junctionId
#  返回类型
#        [ laneId, ... ]
def PanoTrafficApi_GetIncomingLanes(junctionId : str):
    cstr_JunctionId = ctypes.c_char_p(junctionId.encode('utf-8'))
    carray_incomingLaneIds = PanoTrafficAPI_LANE_LIST()

    ret = PanoTrafficInterface.func_GetIncomingLanes(cstr_JunctionId, carray_incomingLaneIds)
    if ret <= 0:
        return []
    else:
        incLanes = []
        for i in range(ret):
            incLanes.append(str(carray_incomingLaneIds[i].value, 'utf-8'))
        return incLanes


#  获得交叉口内部车道列表
#  [定义] junctionId
#  返回类型
#        [ laneId, ... ]
def PanoTrafficApi_GetInternalLanes(junctionId : str):
    cstr_JunctionId = ctypes.c_char_p(junctionId.encode('utf-8'))
    carray_internalLaneIds = PanoTrafficAPI_LANE_LIST()

    ret = PanoTrafficInterface.func_GetInternalLanes(cstr_JunctionId, carray_internalLaneIds)
    if ret <= 0:
        return []
    else:
        intLanes = []
        for i in range(ret):
            intLanes.append(str(carray_internalLaneIds[i].value, 'utf-8'))
        return intLanes


#  获得车道长度
#  [定义] laneId
#  返回类型
#        [double] 长度  单位:m
def PanoTrafficApi_GetLaneLength(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    length = PanoTrafficInterface.func_GetLaneLength(cstr_LaneId)
    return length


#  获得车道宽度
#  [定义] laneId
#  返回类型
#        [double] 宽度  单位:m
def PanoTrafficApi_GetLaneWidth(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    width = PanoTrafficInterface.func_GetLaneWidth(cstr_LaneId)
    return width


#  获得车道是否允许车辆行驶
#  [定义] laneId
#  返回类型
#        [int] 0：不允许车辆行驶；  1：允许车辆行驶
def PanoTrafficApi_GetIsVehicleAllowed(laneId : str):
    cstr_LaneId = ctypes.c_char_p(laneId.encode('utf-8'))
    ret = PanoTrafficInterface.func_GetIsVehicleAllowed(cstr_LaneId)
    return ret






class PanoTrafficInterface:


    #  初始化API，在ModelInit中必须要调用一下
    def PanoTrafficApi_Init(appType : str, appName : str):
        if PanoTrafficInterface.str_AppName != "" or PanoTrafficInterface.str_AppType != "":
            return
        PanoTrafficInterface.str_AppType = appType
        PanoTrafficInterface.str_AppName = appName

        ##### Load PanoTrafficAPI.
        pathPanoSimDatabase = os.environ["PanoSimDatabaseHome"]

        PanoTrafficInterface.str_LoadedApiPath = pathPanoSimDatabase + "\\Plugin\\" + PanoTrafficInterface.str_AppType + "\\" + PanoTrafficInterface.str_AppName + ".dll"
        if ctypes.GetLastError() == 0:
            print("Info: PanoTrafficApi 1.15.1 is loaded successfully. [", PanoTrafficInterface.str_LoadedApiPath, "]")
        else:
            print("Error: Cannot find ", PanoTrafficInterface.str_LoadedApiPath, "!!!")

        if PanoTrafficInterface.str_LoadedApiPath != "":
            PanoTrafficApiDll = ctypes.WinDLL(PanoTrafficInterface.str_LoadedApiPath)

            ##### function importer.
            # func_HelloTest = JunctionDllLib.myTrafficJunctionAPI_HelloTest
            PanoTrafficInterface.func_ChangeLane = PanoTrafficApiDll.PanoTrafficApi_ChangeLane
            PanoTrafficInterface.func_GetVehicleIsLaneChanging = PanoTrafficApiDll.PanoTrafficApi_GetVehicleIsLaneChanging
            PanoTrafficInterface.func_GetNextSafetySpeed = PanoTrafficApiDll.PanoTrafficApi_GetNextSafetySpeed
            PanoTrafficInterface.func_CreateObjectInLaneSL = PanoTrafficApiDll.PanoTrafficApi_CreateObjectInLaneSL
            PanoTrafficInterface.func_MoveObjectBySpeed = PanoTrafficApiDll.PanoTrafficApi_MoveObjectBySpeed
            PanoTrafficInterface.func_GetRealLeader = PanoTrafficApiDll.PanoTrafficApi_GetLeaderVehicle
            PanoTrafficInterface.func_GetFoeInfo = PanoTrafficApiDll.PanoTrafficApi_GetFoeVehiclesInJunction
            PanoTrafficInterface.func_SetObjectRouteByLaneList = PanoTrafficApiDll.PanoTrafficApi_SetObjectRoutesByLaneList
            PanoTrafficInterface.func_SetVehicleRouteByDirections = PanoTrafficApiDll.PanoTrafficApi_SetVehicleRoutesByDirections
            PanoTrafficInterface.func_SetJunctionParameters = PanoTrafficApiDll.PanoTrafficApi_SetJunctionApproachThreshold
            PanoTrafficInterface.func_GetTrafficLightInfo = PanoTrafficApiDll.PanoTrafficApi_GetNextTrafficLightInfo
            PanoTrafficInterface.func_SetConstantTrafficFlowInLane = PanoTrafficApiDll.PanoTrafficApi_SetupConstantTrafficFlowInLane

            PanoTrafficInterface.func_SetObjectAgentControl = PanoTrafficApiDll.PanoTrafficApi_SetObjectAgentControl
            PanoTrafficInterface.func_GetObjectAgentControl = PanoTrafficApiDll.PanoTrafficApi_GetObjectAgentControl
            PanoTrafficInterface.func_SetMultiDisturbanceAgentControl = PanoTrafficApiDll.PanoTrafficApi_SetMultiDisturbanceAgentControl

            ##### function importer 2022-09-05
            PanoTrafficInterface.func_SetupConstantTrafficFlowInLane = PanoTrafficApiDll.PanoTrafficApi_SetupConstantTrafficFlowInLane
            PanoTrafficInterface.func_DeleteObject = PanoTrafficApiDll.PanoTrafficApi_DeleteObject
            PanoTrafficInterface.func_ChangeRoute = PanoTrafficApiDll.PanoTrafficApi_ChangeRoute
            PanoTrafficInterface.func_GetObjectLane = PanoTrafficApiDll.PanoTrafficApi_GetObjectLane
            PanoTrafficInterface.func_GetObjectList = PanoTrafficApiDll.PanoTrafficApi_GetObjectList
            PanoTrafficInterface.func_GetVehicleListInLane = PanoTrafficApiDll.PanoTrafficApi_GetVehicleListInLane
            PanoTrafficInterface.func_GetVehicleListInLaneWithinRange = PanoTrafficApiDll.PanoTrafficApi_GetVehicleListInLaneWithinRange
            PanoTrafficInterface.func_GetDistanceFromLaneStart = PanoTrafficApiDll.PanoTrafficApi_GetDistanceFromLaneStart
            PanoTrafficInterface.func_GetDistanceToLaneEnd = PanoTrafficApiDll.PanoTrafficApi_GetDistanceToLaneEnd
            PanoTrafficInterface.func_GetLateralOffset = PanoTrafficApiDll.PanoTrafficApi_GetLateralOffset
            PanoTrafficInterface.func_GetVehicleJunction = PanoTrafficApiDll.PanoTrafficApi_GetVehicleJunction
            PanoTrafficInterface.func_GetObjectYaw = PanoTrafficApiDll.PanoTrafficApi_GetObjectYaw
            PanoTrafficInterface.func_GetObjectXY = PanoTrafficApiDll.PanoTrafficApi_GetObjectXY
            PanoTrafficInterface.func_GetObjectSpeed = PanoTrafficApiDll.PanoTrafficApi_GetObjectSpeed
            PanoTrafficInterface.func_GetObjectAccel = PanoTrafficApiDll.PanoTrafficApi_GetObjectAccel
            PanoTrafficInterface.func_GetObjectTotalDistance = PanoTrafficApiDll.PanoTrafficApi_GetObjectTotalDistance
            PanoTrafficInterface.func_GetLeaderVehicleInLane = PanoTrafficApiDll.PanoTrafficApi_GetLeaderVehicleInLane
            PanoTrafficInterface.func_GetLeftLeaderVehicle = PanoTrafficApiDll.PanoTrafficApi_GetLeftLeaderVehicle
            PanoTrafficInterface.func_GetRightLeaderVehicle = PanoTrafficApiDll.PanoTrafficApi_GetRightLeaderVehicle
            PanoTrafficInterface.func_GetLeftFollowerVehicle = PanoTrafficApiDll.PanoTrafficApi_GetLeftFollowerVehicle
            PanoTrafficInterface.func_GetRightFollowerVehicle = PanoTrafficApiDll.PanoTrafficApi_GetRightFollowerVehicle
            PanoTrafficInterface.func_GetFollowerVehicle = PanoTrafficApiDll.PanoTrafficApi_GetFollowerVehicle
            PanoTrafficInterface.func_GetLongitudinalDistance = PanoTrafficApiDll.PanoTrafficApi_GetLongitudinalDistance
            PanoTrafficInterface.func_GetLateralDistance = PanoTrafficApiDll.PanoTrafficApi_GetLateralDistance
            PanoTrafficInterface.func_IsLeftLaneExist = PanoTrafficApiDll.PanoTrafficApi_IsLeftLaneExist
            PanoTrafficInterface.func_IsRightLaneExist = PanoTrafficApiDll.PanoTrafficApi_IsRightLaneExist
            PanoTrafficInterface.func_GetLeftLaneId = PanoTrafficApiDll.PanoTrafficApi_GetLeftLaneId
            PanoTrafficInterface.func_GetRightLaneId = PanoTrafficApiDll.PanoTrafficApi_GetRightLaneId
            PanoTrafficInterface.func_GetJunctionList = PanoTrafficApiDll.PanoTrafficApi_GetJunctionList
            PanoTrafficInterface.func_GetValidDirections = PanoTrafficApiDll.PanoTrafficApi_GetValidDirections
            PanoTrafficInterface.func_GetNextLanes = PanoTrafficApiDll.PanoTrafficApi_GetNextLanes
            PanoTrafficInterface.func_GetLaneToJunction = PanoTrafficApiDll.PanoTrafficApi_GetLaneToJunction
            PanoTrafficInterface.func_GetLaneFromJunction = PanoTrafficApiDll.PanoTrafficApi_GetLaneFromJunction
            PanoTrafficInterface.func_GetIsDeadEnd = PanoTrafficApiDll.PanoTrafficApi_GetIsDeadEnd
            PanoTrafficInterface.func_GetIsInternalLane = PanoTrafficApiDll.PanoTrafficApi_GetIsInternalLane
            PanoTrafficInterface.func_GetObjectType = PanoTrafficApiDll.PanoTrafficApi_GetObjectType
            PanoTrafficInterface.func_GetDriverType = PanoTrafficApiDll.PanoTrafficApi_GetDriverType
            PanoTrafficInterface.func_GetVehicleParameter = PanoTrafficApiDll.PanoTrafficApi_GetVehicleParameter

            ##### function importer 2022-09-15
            PanoTrafficInterface.func_MoveObjectByXY = PanoTrafficApiDll.PanoTrafficApi_MoveObjectByXY
            PanoTrafficInterface.func_GetLanePoints = PanoTrafficApiDll.PanoTrafficApi_GetLanePoints
            PanoTrafficInterface.func_GetLaneFrictionCoeff = PanoTrafficApiDll.PanoTrafficApi_GetLaneFrictionCoeff
            PanoTrafficInterface.func_GetLaneCurveCoeff = PanoTrafficApiDll.PanoTrafficApi_GetLaneCurveCoeff
            PanoTrafficInterface.func_GetLaneSlopeAngle = PanoTrafficApiDll.PanoTrafficApi_GetLaneSlopeAngle
            PanoTrafficInterface.func_GetScenarioVisibility = PanoTrafficApiDll.PanoTrafficApi_GetScenarioVisibility

            ##### function importer 2022-09-22
            PanoTrafficInterface.func_GetSimulationCurTimeStamp = PanoTrafficApiDll.PanoTrafficApi_GetSimulationCurTimeStamp
            PanoTrafficInterface.func_GetSimulationTimeStep = PanoTrafficApiDll.PanoTrafficApi_GetSimulationTimeStep

            ##### function importer 2022-09-28
            PanoTrafficInterface.func_GetInternalLanes = PanoTrafficApiDll.PanoTrafficApi_GetInternalLanes
            PanoTrafficInterface.func_GetIncomingLanes = PanoTrafficApiDll.PanoTrafficApi_GetIncomingLanes
            PanoTrafficInterface.func_GetLaneLength = PanoTrafficApiDll.PanoTrafficApi_GetLaneLength

            ##### function importer 2022-10-07
            PanoTrafficInterface.func_CreateVehicleRelated = PanoTrafficApiDll.PanoTrafficApi_CreateVehicleRelated
            
            ##### function importer 2022-11-04
            PanoTrafficInterface.func_CreateObjectXY = PanoTrafficApiDll.PanoTrafficApi_CreateObjectXY
            PanoTrafficInterface.func_MoveObjectByXYYaw = PanoTrafficApiDll.PanoTrafficApi_MoveObjectByXYYaw

            ##### function importer 2022-11-24
            PanoTrafficInterface.func_GetLaneWidth = PanoTrafficApiDll.PanoTrafficApi_GetLaneWidth
            PanoTrafficInterface.func_GetIsVehicleAllowed = PanoTrafficApiDll.PanoTrafficApi_GetIsVehicleAllowed

            ##### function importer 2023-04-19
            PanoTrafficInterface.func_MoveObjectBySpeedAcc = PanoTrafficApiDll.PanoTrafficApi_MoveObjectBySpeedAcc
            PanoTrafficInterface.func_MoveObjectByLaneOffset = PanoTrafficApiDll.PanoTrafficApi_MoveObjectByLateralOffset
            PanoTrafficInterface.func_GetRoute = PanoTrafficApiDll.PanoTrafficApi_GetRoute

            ##### function importer 2023-04-22
            PanoTrafficInterface.func_GetObjectControlTimeInterval = PanoTrafficApiDll.PanoTrafficApi_GetObjectAgentControlTimeInterval

            ##### function interface declaration.
            PanoTrafficInterface.func_ChangeLane.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_double]
            PanoTrafficInterface.func_ChangeLane.restype = ctypes.c_int

            PanoTrafficInterface.func_GetVehicleIsLaneChanging.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetVehicleIsLaneChanging.restype = ctypes.c_int

            PanoTrafficInterface.func_GetNextSafetySpeed.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetNextSafetySpeed.restype = ctypes.c_double

            PanoTrafficInterface.func_CreateObjectInLaneSL.argtypes = [ctypes.c_char_p, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_int, ctypes.c_int]
            PanoTrafficInterface.func_CreateObjectInLaneSL.restype = ctypes.c_int

            PanoTrafficInterface.func_MoveObjectBySpeed.argtypes = [ctypes.c_int, ctypes.c_double, ctypes.c_double]
            PanoTrafficInterface.func_MoveObjectBySpeed.restype = ctypes.c_int

            PanoTrafficInterface.func_GetRealLeader.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetRealLeader.restype = PyLeaderInfo

            PanoTrafficInterface.func_GetFoeInfo.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetFoeInfo.restype = PyCollisionPointInfo

            PanoTrafficInterface.func_SetObjectRouteByLaneList.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_char * MAX_LANE_NAME_LENGTH)]
            PanoTrafficInterface.func_SetObjectRouteByLaneList.restype = ctypes.c_int

            PanoTrafficInterface.func_SetVehicleRouteByDirections.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
            PanoTrafficInterface.func_SetVehicleRouteByDirections.restype = ctypes.c_int

            PanoTrafficInterface.func_SetJunctionParameters.argtypes = [ctypes.c_double, ctypes.c_double]
            PanoTrafficInterface.func_SetJunctionParameters.restype = ctypes.c_int

            PanoTrafficInterface.func_GetTrafficLightInfo.argtypes = [ctypes.c_int, ctypes.c_int]
            PanoTrafficInterface.func_GetTrafficLightInfo.restype = PyTrafficLightInfo

            PanoTrafficInterface.func_SetConstantTrafficFlowInLane.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_double, ctypes.c_double]
            PanoTrafficInterface.func_SetConstantTrafficFlowInLane.restype = ctypes.c_int

            PanoTrafficInterface.func_SetObjectAgentControl.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_double]
            PanoTrafficInterface.func_SetObjectAgentControl.restype = ctypes.c_int

            PanoTrafficInterface.func_GetObjectAgentControl.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetObjectAgentControl.restype = ctypes.c_int

            PanoTrafficInterface.func_SetMultiDisturbanceAgentControl.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_SetMultiDisturbanceAgentControl.restype = ctypes.c_int

            ######  function parameter declarations 2022-09-05
            PanoTrafficInterface.func_SetupConstantTrafficFlowInLane.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_double, ctypes.c_double]
            PanoTrafficInterface.func_SetupConstantTrafficFlowInLane.restype = ctypes.c_int

            PanoTrafficInterface.func_DeleteObject.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_DeleteObject.restype = ctypes.c_int

            PanoTrafficInterface.func_ChangeRoute.argtypes = [ctypes.c_int, ctypes.c_int]
            PanoTrafficInterface.func_ChangeRoute.restype = ctypes.c_int

            PanoTrafficInterface.func_GetObjectLane.argtypes = [ctypes.c_int, ctypes.c_char_p]
            PanoTrafficInterface.func_GetObjectLane.restype = ctypes.c_int

            PanoTrafficInterface.func_GetObjectList.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
            PanoTrafficInterface.func_GetObjectList.restype = ctypes.c_int

            PanoTrafficInterface.func_GetVehicleListInLane.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
            PanoTrafficInterface.func_GetVehicleListInLane.restype = ctypes.c_int

            PanoTrafficInterface.func_GetVehicleListInLaneWithinRange.argtypes = [ctypes.c_char_p, ctypes.c_double, ctypes.c_double, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
            PanoTrafficInterface.func_GetVehicleListInLaneWithinRange.restype = ctypes.c_int

            PanoTrafficInterface.func_GetDistanceFromLaneStart.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetDistanceFromLaneStart.restype = ctypes.c_double

            PanoTrafficInterface.func_GetDistanceToLaneEnd.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetDistanceToLaneEnd.restype = ctypes.c_double

            PanoTrafficInterface.func_GetLateralOffset.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetLateralOffset.restype = ctypes.c_double

            PanoTrafficInterface.func_GetVehicleJunction.argtypes = [ctypes.c_int, ctypes.c_char_p]
            PanoTrafficInterface.func_GetVehicleJunction.restype = ctypes.c_int

            PanoTrafficInterface.func_GetObjectYaw.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetObjectYaw.restype = ctypes.c_double

            PanoTrafficInterface.func_GetObjectXY.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
            PanoTrafficInterface.func_GetObjectXY.restype = ctypes.c_int

            PanoTrafficInterface.func_GetObjectSpeed.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetObjectSpeed.restype = ctypes.c_double

            PanoTrafficInterface.func_GetObjectAccel.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetObjectAccel.restype = ctypes.c_double

            PanoTrafficInterface.func_GetObjectTotalDistance.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetObjectTotalDistance.restype = ctypes.c_double

            PanoTrafficInterface.func_GetLeftLeaderVehicle.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetLeftLeaderVehicle.restype = ctypes.c_int

            PanoTrafficInterface.func_GetRightLeaderVehicle.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetRightLeaderVehicle.restype = ctypes.c_int

            PanoTrafficInterface.func_GetLeftFollowerVehicle.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetLeftFollowerVehicle.restype = ctypes.c_int

            PanoTrafficInterface.func_GetRightFollowerVehicle.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetRightFollowerVehicle.restype = ctypes.c_int

            PanoTrafficInterface.func_GetFollowerVehicle.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetFollowerVehicle.restype = ctypes.c_int

            PanoTrafficInterface.func_GetLeaderVehicleInLane.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetLeaderVehicleInLane.restype = ctypes.c_int

            PanoTrafficInterface.func_GetLongitudinalDistance.argtypes = [ctypes.c_int, ctypes.c_int]
            PanoTrafficInterface.func_GetLongitudinalDistance.restype = ctypes.c_double

            PanoTrafficInterface.func_GetLateralDistance.argtypes = [ctypes.c_int, ctypes.c_int]
            PanoTrafficInterface.func_GetLateralDistance.restype = ctypes.c_double

            PanoTrafficInterface.func_IsLeftLaneExist.argtypes = [ctypes.c_char_p]
            PanoTrafficInterface.func_IsLeftLaneExist.restype = ctypes.c_int

            PanoTrafficInterface.func_IsRightLaneExist.argtypes = [ctypes.c_char_p]
            PanoTrafficInterface.func_IsRightLaneExist.restype = ctypes.c_int

            PanoTrafficInterface.func_GetLeftLaneId.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
            PanoTrafficInterface.func_GetLeftLaneId.restype = ctypes.c_int

            PanoTrafficInterface.func_GetRightLaneId.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
            PanoTrafficInterface.func_GetRightLaneId.restype = ctypes.c_int

            PanoTrafficInterface.func_GetJunctionList.argtypes = [ctypes.POINTER(ctypes.c_char * MAX_LANE_NAME_LENGTH)]
            PanoTrafficInterface.func_GetJunctionList.restype = ctypes.c_int

            PanoTrafficInterface.func_GetValidDirections.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
            PanoTrafficInterface.func_GetValidDirections.restype = ctypes.c_int

            PanoTrafficInterface.func_GetNextLanes.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char * MAX_LANE_NAME_LENGTH)]
            PanoTrafficInterface.func_GetNextLanes.restype = ctypes.c_int

            PanoTrafficInterface.func_GetLaneToJunction.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
            PanoTrafficInterface.func_GetLaneToJunction.restype = ctypes.c_int

            PanoTrafficInterface.func_GetLaneFromJunction.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
            PanoTrafficInterface.func_GetLaneFromJunction.restype = ctypes.c_int

            PanoTrafficInterface.func_GetIsDeadEnd.argtypes = [ctypes.c_char_p]
            PanoTrafficInterface.func_GetIsDeadEnd.restype = ctypes.c_int

            PanoTrafficInterface.func_GetIsInternalLane.argtypes = [ctypes.c_char_p]
            PanoTrafficInterface.func_GetIsInternalLane.restype = ctypes.c_int

            PanoTrafficInterface.func_GetObjectType.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetObjectType.restype = ctypes.c_int

            PanoTrafficInterface.func_GetDriverType.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetDriverType.restype = ctypes.c_int

            PanoTrafficInterface.func_GetVehicleParameter.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetVehicleParameter.restype = PyVehicleParameter

            ##### function importer 2022-09-15
            PanoTrafficInterface.func_MoveObjectByXY.argtypes = [ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double]
            PanoTrafficInterface.func_MoveObjectByXY.restype = ctypes.c_int

            PanoTrafficInterface.func_GetLanePoints.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_double * MAX_LANE_POINT_COUNT)]
            PanoTrafficInterface.func_GetLanePoints.restype = ctypes.c_int

            PanoTrafficInterface.func_GetLaneFrictionCoeff.argtypes = [ctypes.c_char_p]
            PanoTrafficInterface.func_GetLaneFrictionCoeff.restype = ctypes.c_double

            PanoTrafficInterface.func_GetLaneCurveCoeff.argtypes = [ctypes.c_int, ctypes.c_double]
            PanoTrafficInterface.func_GetLaneCurveCoeff.restype = ctypes.c_double

            PanoTrafficInterface.func_GetLaneSlopeAngle.argtypes = [ctypes.c_int, ctypes.c_double]
            PanoTrafficInterface.func_GetLaneSlopeAngle.restype = ctypes.c_double

            PanoTrafficInterface.func_GetScenarioVisibility.argtypes = []
            PanoTrafficInterface.func_GetScenarioVisibility.restype = ctypes.c_double

            ##### function importer 2022-09-22
            PanoTrafficInterface.func_GetSimulationCurTimeStamp.argtypes = []
            PanoTrafficInterface.func_GetSimulationCurTimeStamp.restype = ctypes.c_double

            PanoTrafficInterface.func_GetSimulationTimeStep.argtypes = []
            PanoTrafficInterface.func_GetSimulationTimeStep.restype = ctypes.c_double

            ##### function importer 2022-09-28
            PanoTrafficInterface.func_GetInternalLanes.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_char * MAX_LANE_NAME_LENGTH)]
            PanoTrafficInterface.func_GetInternalLanes.restype = ctypes.c_int

            PanoTrafficInterface.func_GetIncomingLanes.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_char * MAX_LANE_NAME_LENGTH)]
            PanoTrafficInterface.func_GetIncomingLanes.restype = ctypes.c_int

            PanoTrafficInterface.func_GetLaneLength.argtypes = [ctypes.c_char_p]
            PanoTrafficInterface.func_GetLaneLength.restype = ctypes.c_double

            ##### function importer 2022-10-07
            PanoTrafficInterface.func_CreateVehicleRelated.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_int, ctypes.c_int]
            PanoTrafficInterface.func_CreateVehicleRelated.restype = ctypes.c_int
            
            ##### function importer 2022-11-04
            PanoTrafficInterface.func_CreateObjectXY.argtypes = [ctypes.c_char_p, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_int, ctypes.c_int]
            PanoTrafficInterface.func_CreateObjectXY.restype = ctypes.c_int
            
            PanoTrafficInterface.func_MoveObjectByXYYaw.argtypes = [ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
            PanoTrafficInterface.func_MoveObjectByXYYaw.restype = ctypes.c_int

            ##### function importer 2022-11-24
            PanoTrafficInterface.func_GetLaneWidth.argtypes = [ctypes.c_char_p]
            PanoTrafficInterface.func_GetLaneWidth.restype = ctypes.c_double

            PanoTrafficInterface.func_GetIsVehicleAllowed.argtypes = [ctypes.c_char_p]
            PanoTrafficInterface.func_GetIsVehicleAllowed.restype = ctypes.c_int

            ##### function importer 2023-04-19
            PanoTrafficInterface.func_MoveObjectBySpeedAcc.argtypes = [ctypes.c_int, ctypes.c_double, ctypes.c_double]
            PanoTrafficInterface.func_MoveObjectBySpeedAcc.restype = ctypes.c_int

            PanoTrafficInterface.func_MoveObjectByLaneOffset.argtypes = [ctypes.c_int, ctypes.c_double, ctypes.c_double]
            PanoTrafficInterface.func_MoveObjectByLaneOffset.restype = ctypes.c_int

            PanoTrafficInterface.func_GetRoute.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetRoute.restype = ctypes.c_int

            PanoTrafficInterface.func_GetObjectControlTimeInterval.argtypes = [ctypes.c_int]
            PanoTrafficInterface.func_GetObjectControlTimeInterval.restype = ctypes.c_double

            
            

    ### Class static member
    # PanoTrafficApiDll = 0

    func_ChangeLane = 0
    func_GetVehicleIsLaneChanging = 0
    func_GetNextSafetySpeed = 0
    func_CreateObjectInLaneSL = 0
    func_MoveObjectBySpeed = 0
    func_GetRealLeader = 0
    func_GetFoeInfo = 0
    func_SetObjectRouteByLaneList = 0
    func_SetVehicleRouteByDirections = 0
    func_SetJunctionParameters = 0
    func_GetTrafficLightInfo = 0
    func_SetConstantTrafficFlowInLane = 0

    func_SetObjectAgentControl = 0
    func_GetObjectAgentControl = 0
    func_SetMultiDisturbanceAgentControl = 0

    func_SetupConstantTrafficFlowInLane = 0
    func_DeleteObject = 0
    func_ChangeRoute = 0
    func_GetObjectLane = 0
    func_GetObjectList = 0
    func_GetVehicleListInLane = 0
    func_GetVehicleListInLaneWithinRange = 0
    func_GetDistanceFromLaneStart = 0
    func_GetLateralOffset = 0
    func_GetVehicleJunction = 0
    func_GetObjectYaw = 0
    func_GetObjectXY = 0
    func_GetObjectSpeed = 0
    func_GetObjectAccel = 0
    func_GetObjectTotalDistance = 0
    func_GetLeaderVehicleInLane = 0
    func_GetLeftLeaderVehicle = 0
    func_GetRightLeaderVehicle = 0
    func_GetLeftFollowerVehicle = 0
    func_GetRightFollowerVehicle = 0
    func_GetFollowerVehicle = 0
    func_GetLongitudinalDistance = 0
    func_GetLateralDistance = 0
    func_IsLeftLaneExist = 0
    func_IsRightLaneExist = 0
    func_GetLeftLaneId = 0
    func_GetRightLaneId = 0
    func_GetJunctionList = 0
    func_GetValidDirections = 0
    func_GetNextLanes = 0
    func_GetLaneToJunction = 0
    func_GetLaneFromJunction = 0
    func_GetIsDeadEnd = 0
    func_GetIsInternalLane = 0
    func_GetObjectType = 0
    func_GetDriverType = 0
    func_GetVehicleParameter = 0

    func_MoveObjectByXY = 0
    func_GetLanePoints = 0
    func_GetLaneFrictionCoeff = 0
    func_GetLaneCurveCoeff = 0
    func_GetLaneSlopeAngle = 0
    func_GetScenarioVisibility = 0

    func_GetSimulationCurTimeStamp = 0
    func_GetSimulationTimeStep = 0
    func_GetObjectControlTimeInterval = 0

    func_GetInternalLanes = 0
    func_GetIncomingLanes = 0
    func_GetLaneLength = 0

    func_CreateVehicleRelated = 0
    func_CreateObjectXY = 0
    func_MoveObjectByXYYaw = 0

    func_GetLaneWidth = 0
    func_GetIsVehicleAllowed = 0

    func_MoveObjectBySpeedAcc = 0
    func_MoveObjectByLaneOffset = 0
    func_GetRoute = 0

    str_AppName = ""
    str_AppType = ""
    str_LoadedApiPath = ""





