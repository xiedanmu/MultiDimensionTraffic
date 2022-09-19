# 脚本描述：     该文件用于拓展PanoSimAPI无法提供的一些公用函数
# 版本：        v0.1
# 作者：        白雪松
# 说明：        创建接口，完成基本实现
# 日期：        2021-xx-xx


from TrafficModelInterface import *
#xxfrom myGetNextLane import *
# import myGetLaneShapeBXS
#xxfrom myGetLaneShapeBXS import *
# import myGetLaneShapeBXS
from Library.package_platformAPI.PanoSimTrafficAPI2 import *
#xxfrom scipy.interpolate import UnivariateSpline
import numpy as np



MY_PANOSIM_INVALID_VALUE = -1073741824

# API BXS新增
def myGetOffset(vehID):
    return getVehicleLateralOffset(vehID)

def myGetVehicleList():
    return getVehicleList()

def myConflict(ConflictPoint,junctionID):
    ConflictPointLane = [['gneE0_0', ':gneJ1_1_0'],['gneE3_0', ':gneJ1_0_0']]#todo
    vehList = getVehicleList()
    vehList1 = getLaneVehicles(ConflictPointLane[0][0])
    for i in vehList1:
        if myGetDistanceToLaneEnd(i) > 20:
            pp = vehList1.index(i)
            vehList.pop(pp)
    vehList2 = []
    vehList4 = []
    for i in vehList:
        if myGetVehicleX(i) >57.6 and myGetVehicleX(i) < ConflictPoint[0][0]:# #todo, 需要接口函数判断节点边界
            if myGetLaneID(i) == ':gneJ1_1_0': #todo
                vehList2.append(i)
            elif myGetLaneID(i) == ':gneJ1_0_0':#todo
                vehList4.append(i)
    vehList3 = getLaneVehicles(ConflictPointLane[1][0])
    for i in vehList3:
        if myGetDistanceToLaneEnd(i) > 20:
            pp = vehList3.index(i)
            vehList.pop(pp)

    if vehList1 and vehList3:
        ConflictPointQueue = [[[vehList1[-1]], vehList2],[[vehList3[-1]],vehList4]]
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
        dis ={}
        for i in ConflictPointQueue[0][1]:
            dis[i] =  myGetVehicleX(i)
        dis = sorted(dis.items(), key=lambda item: item[1])
        tem = []
        for i in range(len(dis)):
            tem.append(dis[i][0])
        for i in tem:
            ConflictPointQueueFin1.append(i)

    if ConflictPointQueue[1][0]:
        ConflictPointQueueFin2.append(ConflictPointQueue[1][0][0])
    if ConflictPointQueue[1][1]:
        dis ={}
        for i in ConflictPointQueue[1][1]:
            dis[i] =  myGetVehicleX(i)
        dis = sorted(dis.items(), key=lambda item: item[1])
        tem = []
        for i in range(len(dis)):
            tem.append(dis[i][0])
        for i in tem:
            ConflictPointQueueFin2.append(i)

    ConflictPointQueueFin = [ConflictPointQueueFin1,ConflictPointQueueFin2]

    # print('2222',ConflictPointQueue)
    # print('6666',ConflictPointQueueFin)
    # for i in ConflictPointQueueFin[0]:
    #     print("7777",myGetVehicleX(i))
    # for i in ConflictPointQueueFin[1]:
    #     print("8888", myGetVehicleX(i))

    # 获取冲突点前前的两个车队序列【0：主干道序列，1：匝道系列】
    return ConflictPointQueueFin

def myConflict1(ConflictPoint,junctionID):
    ConflictPointLane = [['gneE0_0', ':gneJ1_1_0'],['gneE3_0', ':gneJ1_0_0']]#todo
    vehList = getVehicleList()

    vehList2 = []
    vehList4 = []
    for i in vehList:
        if myGetVehicleX(i) >57.6 and myGetVehicleX(i) < ConflictPoint[0][0]:# #todo, 需要接口函数判断节点边界
            if myGetLaneID(i) == ':gneJ1_1_0': #todo
                vehList2.append(i)
            elif myGetLaneID(i) == ':gneJ1_0_0':#todo
                vehList4.append(i)


    ConflictPointQueue = [[vehList2], [vehList4]]

    # [[[9], []], [[7], []]]
    # [[9], [7]]
    ConflictPointQueueFin1 = []
    ConflictPointQueueFin2 = []

    if ConflictPointQueue[0][0]:
        ConflictPointQueueFin1.append(ConflictPointQueue[0][0][0])
    if ConflictPointQueue[0][1]:
        dis ={}
        for i in ConflictPointQueue[0][1]:
            dis[i] =  myGetVehicleX(i)
        dis = sorted(dis.items(), key=lambda item: item[1])
        tem = []
        for i in range(len(dis)):
            tem.append(dis[i][0])
        for i in tem:
            ConflictPointQueueFin1.append(i)

    if ConflictPointQueue[1][0]:
        ConflictPointQueueFin2.append(ConflictPointQueue[1][0][0])
    if ConflictPointQueue[1][1]:
        dis ={}
        for i in ConflictPointQueue[1][1]:
            dis[i] =  myGetVehicleX(i)
        dis = sorted(dis.items(), key=lambda item: item[1])
        tem = []
        for i in range(len(dis)):
            tem.append(dis[i][0])
        for i in tem:
            ConflictPointQueueFin2.append(i)

    ConflictPointQueueFin = [ConflictPointQueueFin1,ConflictPointQueueFin2]

    # print('2222',ConflictPointQueue)
    # print('6666',ConflictPointQueueFin)
    # for i in ConflictPointQueueFin[0]:
    #     print("7777",myGetVehicleX(i))
    # for i in ConflictPointQueueFin[1]:
    #     print("8888", myGetVehicleX(i))

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

def myIDM(vehID, leadVehID, myLeaderDis):
    curSpeed = myGetVehicleSpeed(vehID)
    vDesired = 120 / 3.6
    delta = 4
    thwDesired = 1.5
    s0 = 2
    aMax = 2
    bCom = 2
    if leadVehID < 0:
        acc = aMax * (1. - pow((curSpeed / vDesired), delta))
    else:
        deltaS = myLeaderDis
        leadSpeed = myGetVehicleSpeed(leadVehID)
        deltaV = curSpeed - leadSpeed
        if deltaS < 0.5:
            deltaS = 0.5

        sStar = s0 + curSpeed * thwDesired + curSpeed * deltaV / (2. * pow(aMax * bCom, 0.5))
        acc = aMax * (1. - pow((curSpeed / vDesired), delta) - pow(sStar / deltaS, 2.))
    return acc

# BXS
def myGetFromJunction(laneID):
    return getFromJunction(laneID)

def myGetNextJunction(laneID):
    return getToJunction(laneID)

def myCheckEndJunction(lastJunctionID):
    incomingEdge = {}
    incomingLanes = getIncomingLanes(lastJunctionID)
    for incLane in incomingLanes:
        index = incLane.find('_')
        temEdge = incLane[0:index]
        # .pop(index:-1)
        # temEdge = incLane[3:5]
        incomingEdge[temEdge] = temEdge
    ee = len(incomingEdge)
    if len(incomingEdge) < 2:
        return True
    else:
        return False

def myGetVehicleInLane(laneID): #返回是按照s从小到大排列的
    vehList = getVehicleList()
    vheInLane= []
    if not myCheckInternalLane(laneID):
        # getLaneVehicles（）的返回是按照s从小到大排列的
        vheInLane = myGetVehiclesInLane(laneID)
    else:
        for i in vehList:
            if myGetLaneID(i) is laneID:
                vheInLane.append(i)
        if len(vheInLane) > 1:
            vheLiTem = {}
            vehLi = []
            for j in vheInLane:
                vheLiTem[j] = myGetDistanceFromLaneStart(j)
            dis = sorted(vheLiTem.items(), key=lambda item: item[1])
            tem = []
            for i in range(len(dis)):
                tem.append(dis[i][0])
            for i in tem:
                vehLi.append(i)
            vheInLane = vehLi
    return vheInLane

def myGetTrafficLightState(laneID, direction):
    return getTrafficLightState(laneID, direction)

def myGetTrafficLightStateTimer(laneID, direction):
    return getTrafficLightStateTimer(laneID, direction)

def myGetVehicleRoute(id):
    return getRoute(id)

def myGetLaneValidDirections(laneID):
    return getValidDirections(laneID)

def myGetLeaderVehicle(id):
    curLane = myGetLaneID(id)
    myVehRoute = myGetVehicleRoute(id)
    myNextLane = myGetNextLane(curLane,myVehRoute)
    myNextNextLane = myGetNextLane(myNextLane)
    curS = myGetDistanceFromLaneStart(id)
    disToEnd = myGetDistanceToLaneEnd(id)

    if not myCheckInternalLane(curLane) and disToEnd > 20:
        myLeader = getLeaderVehicle(id)
        if myLeader >= 0:
            myLeaderDis = myGetDistanceFromLaneStart(myLeader) - myGetDistanceFromLaneStart(id)
        else:
            myLeaderDis = 999
    else:
        if myCheckInternalLane(myNextNextLane): # 节点中内部道路上的节点
            vehList1 = myGetVehicleInLane(myNextLane)
            if len(vehList1) > 0:
                myLeader = vehList1[0]
                myLeaderDis = myGetDistanceToLaneEnd(id) + myGetDistanceFromLaneStart(myLeader)
            else:
                vehList2 = myGetVehicleInLane(myNextNextLane)
                if len(vehList2) > 0:
                    myLeader = vehList2[0]
                    myLeaderDis = myGetDistanceToLaneEnd(id) + myGetDistanceFromLaneStart(myLeader)
                else:
                    myLeader = -1
                    myLeaderDis = 999
        else:
            vehList = myGetVehicleInLane(myNextLane)
            if len(vehList) > 0:
                myLeader = vehList[0]
                myLeaderDis = myGetDistanceToLaneEnd(id) + myGetDistanceFromLaneStart(myLeader)
            else:
                myLeader = -1
                myLeaderDis = 999
    # , myLeaderS
    return myLeader, myLeaderDis

def myGetFollowerVehicle(id):
    # return getFollowerVehicle(id)
    myFollower = getFollowerVehicle(id)
    if myFollower > -1 :
        myFollowerDis =  myGetDistanceFromLaneStart(id) - myGetDistanceFromLaneStart(myFollower)
    else:
        myFollowerDis =999
    return myFollower, myFollowerDis

def myGetLeftLeaderVehicle(id):
    myLeftLeader = getLeftLeaderVehicle(id)
    if myLeftLeader > -1 :
        myLeftLeaderDis =  myGetDistanceFromLaneStart(myLeftLeader) - myGetDistanceFromLaneStart(id)
    else:
        myLeftLeaderDis =999
    return myLeftLeader, myLeftLeaderDis

def myGetLeftFollowerVehicle(id) -> int:
    # return getLeftFollowerVehicle(id)
    myLeftFollower = getLeftFollowerVehicle(id)
    return myLeftFollower

def myGetLeftFollowerVehicleAndDistance(id) -> (int,int):
    # return getLeftFollowerVehicle(id)
    myLeftFollower = getLeftFollowerVehicle(id)
    if myLeftFollower > -1 :
        myLeftFollowerDis =  myGetDistanceFromLaneStart(id) - myGetDistanceFromLaneStart(myLeftFollower)
    else:
        myLeftFollowerDis =999
    return myLeftFollower, myLeftFollowerDis

def myGetRightLeaderVehicle(id):
    # return getRightLeaderVehicle(id)
    myRightLeader = getRightLeaderVehicle(id)
    if myRightLeader > -1 :
        myRightLeaderDis =  myGetDistanceFromLaneStart(myRightLeader) - myGetDistanceFromLaneStart(id)
    else:
        myRightLeaderDis =999
    return myRightLeader, myRightLeaderDis

def myGetRightFollowerVehicle(id) -> int:
    # return getRightFollowerVehicle(id)
    myRightFollower = getRightFollowerVehicle(id)
    return myRightFollower

def myGetRightFollowerVehicleAndDistance(id)->(int,int):
    # return getRightFollowerVehicle(id)
    myRightFollower = getRightFollowerVehicle(id)
    if myRightFollower > -1 :
        myRightFollowerDis =  myGetDistanceFromLaneStart(id) - myGetDistanceFromLaneStart(myRightFollower)
    else:
        myRightFollowerDis =999
    return myRightFollower, myRightFollowerDis

# def myGetLaneShapebxs(laneID):
#     return myGetLaneShapeBXS(laneID)

def myGetLaneStationYaw(laneID, S):
    dd = laneID
    print("laneID", laneID)
    i = isInternalLane(laneID)
        # print("edge")
    myLaneShape = getLaneShape(laneID)

    laneShape = myGetLaneShapeBXS(laneID)

    laneSha = getLaneShapeWithOnlyXY(laneShape)
    LaneStationYaw = laneSha[0][3]
    for i in range(len(laneSha)-1):
        if S > laneSha[i][2] and S < laneSha[i+1][2] :
            LaneStationYaw = laneSha[i][3]
            break
    return LaneStationYaw

def myGetSLFromXY(laneID, x, y):
    # 找到lane的所有点，求（x,y）到各个点的距离，找打距离最小的两个点，向距离最小的两个点做垂线，垂线交点往道路起始点累加获得S，垂线的高是L
    points = myGetLaneShapeBXS
    points = getLaneShapeWithOnlyXY(points)

    def calculateTwoNearestPoint(x,y):
        if len(points) == 0:
            return None, None
        elif len(points) < 3:
            id1 = 0
            id2 = 1
            return id1, id2
        else:
            dis = []
            for j in range(len(points)):
                tem = math.sqrt(math.pow(x-points[j][1],2)+math.pow(y-points[j],2))
                dis.append(tem)
            id1 = dis.index(min(dis))
            if id1 == 0:
                id2 = id1 + 1
            elif id1 == len(points):
                id2 = id1 - 1
            else:
                if dis[id1 + 1] < dis[id1 - 1]:
                    id2 = id1 + 1
                else:
                    id2 = id1 - 1
            return id1, id2

    id1, id2 = calculateTwoNearestPoint(x,y)
    if points[id1][2] > points[id2][2]:
        tem = id1
        id1 = id2
        id2 = tem
    x1 = points[id1][0]
    y1 = points[id1][1]
    x2 = points[id2][0]
    y2 = points[id2][1]
    # 第二点和第一点
    k = (y2 - y1)/(x2 - x1)
    b = (x2*y1 - x1*y2)/(x2-x1)
    thelta1 = math.atan(k)
    # 路径点和第一点
    k1 = (y - y1)/(x - x1)
    b1 = (x*y1 - x1*y)/(x-x1)
    thelta2 = math.atan(k1)
    # thelta2 = 180 * thelta2 / math.pi
    distance = math.sqrt(math.pow(x - x1, 2) + math.pow(y - y1, 2))
    if thelta1 > thelta2: # l 为正
        s = points[id1][2] + distance * math.cos(thelta1 - thelta2)
        l = distance * math.sin(thelta1 - thelta2)
    else:
        s = points[id1][2] + distance * math.cos(thelta1 - thelta2)
        l = -1 * distance * math.sin(thelta1 - thelta2)
    laneAngle = 180 * thelta1 / math.pi
    return s,l

def myGetPointYaw(x,y,vehTraj):
    # 计算轨迹上任意点的angle
    points = vehTraj.m_lTrajectory
    def calculateTwoNearestPoint(x, y, points):
        if len(points) == 0:
            return None, None
        elif len(points) < 3:
            id1 = 0
            id2 = 1
            return id1, id2
        else:
            dis = []
            for j in range(len(points)):
                tem = math.sqrt(math.pow(x - points[j][1], 2) + math.pow(y - points[j], 2))
                dis.append(tem)
            id1 = dis.index(min(dis))
            if id1 == 0:
                id2 = id1 + 1
            elif id1 == len(points):
                id2 = id1 - 1
            else:
                if dis[id1 + 1] < dis[id1 - 1]:
                    id2 = id1 + 1
                else:
                    id2 = id1 - 1
            return id1, id2

    id1, id2 = calculateTwoNearestPoint(x, y)
    x1 = points[id1][0]
    y1 = points[id1][1]
    x2 = points[id2][0]
    y2 = points[id2][1]
    k = (y2 - y1) / (x2 - x1)
    b = (x2 * y1 - x1 * y2) / (x2 - x1)
    thelta1 = math.atan(k)
    thelta1 = 180 * thelta1 / math.pi
    return thelta1

#[i] x,y全局坐标系，4*1
def xxx(x, y, carX, carY, carYaw):
    # 全局坐标系转车辆坐标系
    # print('全局坐标系坐标点')
    # print(ptsx)
    # print(ptsy)
    ptsx = x
    ptsy = y
    ref_x = carX
    ref_y = carY
    ref_yaw = carYaw
    for i in range(len(ptsx)):
        shift_x = ptsx[i] - ref_x
        shift_y = ptsy[i] - ref_y
        ptsx[i] = shift_x * np.cos(0 - ref_yaw) - shift_y * np.sin(0 - ref_yaw)
        ptsy[i] = shift_x * np.sin(0 - ref_yaw) + shift_y * np.cos(0 - ref_yaw)

    # 样条曲线生成
    if all(x <= y for x, y in zip(ptsx[:len(ptsx)], ptsx[1:])):
        # 增序列
        csp = UnivariateSpline(ptsx, ptsy, k=3)
    else:
        # 降序列
        ptsx.reverse()
        ptsy.reverse()
        csp = UnivariateSpline(ptsx, ptsy, k=3)

    x_planning = np.arange(0, 30)
    y_planning = csp(x_planning)
    next_x_vals=[]
    next_y_vals=[]
    for i in range(x_planning.shape[0]):
        x_point = x_planning[i]
        y_point = y_planning[i]

        x_ref = x_point
        y_ref = y_point
        x_point = x_ref * np.cos(ref_yaw) - y_ref * np.sin(ref_yaw)
        y_point = x_ref * np.sin(ref_yaw) + y_ref * np.cos(ref_yaw)

        x_point += ref_x
        y_point += ref_y

        next_x_vals.append(x_point)
        next_y_vals.append(y_point)
    return next_x_vals, next_y_vals


# ob = np.array([[20.0, 10.0],
#                [30.0, 6.0],
#                [30.0, 8.0],
#                [35.0, 8.0],
#                [50.0, 3.0]
#                ])
def check_collision(fpx, fpy, ob) ->bool:
    faTrajCollisionX = []
    faTrajCollisionY = []
    faObCollisionX = []
    faObCollisionY = []
    ROBOT_RADIUS = 0.7
    # pdb.set_trace()
    if ob.size == 0:
        return False
    for i in range(len(ob[:, 0])):
        # Calculate the distance for each trajectory point to the current object
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fpx, fpy)]

        # Check if any trajectory point is too close to the object using the robot radius
        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            # plot(ft.x, ft.y, 'rx')
            faTrajCollisionX.append(fpx)
            faTrajCollisionY.append(fpy)
            # plot(ox, oy, 'yo');
            # pdb.set_trace()
            if ob[i, 0] not in faObCollisionX or ob[i, 1] not in faObCollisionY:
                faObCollisionX.append(ob[i, 0])
                faObCollisionY.append(ob[i, 1])
            return True
    return False

def myPoDF(s,D,W):
    PointX = s
    if D < 0.01:
        D = 0.01
    PointY = W * (6 * math.pow(PointX / D, 5) - 15 * math.pow(PointX / D, 4) + 10 * math.pow(PointX / D, 3))
    return PointY

def myGetVehicleType(id):
    return getVehicleType(id)

# 函数: calculateTwoNearestPoint
# 用途: 寻找一个点集中距离目标点最近的两个点
# [i]: point(x,y)
# [i]: pointSet
# [o]: 两个最近点的下标
def calculateTwoNearestPoint(x, y, points):
    if len(points) == 0:
        return None, None
    elif len(points) < 3:
        id1 = 0
        id2 = 1
        return id1, id2
    else:
        dis = []
        for j in range(len(points)):
            tem = math.sqrt(math.pow(x - points[j][1], 2) + math.pow(y - points[j], 2))
            dis.append(tem)
        id1 = dis.index(min(dis))
        if id1 == 0:
            id2 = id1 + 1
        elif id1 == len(points):
            id2 = id1 - 1
        else:
            if dis[id1 + 1] < dis[id1 - 1]:
                id2 = id1 + 1
            else:
                id2 = id1 - 1
        return id1, id2


def calculate2pointsYaw(x1, y1, x2, y2):
    if x1 is None or y1 is None or x2 is None or y2 is None:
        pass
    # 更正：此处为sumo坐标系，正北为0，顺时针为正
    yaw = 0
    dy = y2 - y1
    dx = x2 - x1
    if dx == 0 and dy > 0:
        yaw = 0
        # yaw = math.pi / 2
    elif dx == 0 and dy < 0:
        yaw = math.pi
    elif dy == 0 and dx > 0:
        yaw = math.pi/2
    elif dy == 0 and dx < 0:
        yaw = 3*math.pi/2
    else:
        if dx > 0:
            yaw = math.pi / 2 - math.atan2(dy, dx)
        else:
            yaw = 3 * math.pi / 2 - math.atan2(dy, dx)
    return yaw