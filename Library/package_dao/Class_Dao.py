# 通过实体属性获取对象的Dao层 data access object
#
#
import math
import random

import numpy as np

# from Library.package_platformAPI.SimPlatformAPI import SimPlatformAPI
from Library.package_platformAPI import PanoSimTrafficAPI2
from Library import PanoTrafficApi_Python as PanoTrafficAPI
from Library.package_entity.Class_Trajectory import Trajectory
from Library.package_entity.Class_Vehicle import Vehicle
from Library.package_entity.Class_Lane import Lane
from Library.package_entity.Class_Internal_Lane import InternalLane
from Library.package_entity.Enum_Types_Of_Road import TypesOfRoad
from Library.package_interface.Algorithm_Interface_Impl import AlgorithmInterfaceImpl
import logging
log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())

AlgorithmInterface = AlgorithmInterfaceImpl

class Dao:
    """与userdata互动的data access object层"""
    vehicle_dictionary = {}
    trajectory_dictionary = {}
    lane_dictionary = {}  # 会有lane类实例 或者internal_lane类实例

    dicLaneShapes = {}
    dicLaneFromLanes = {}
    dicLaneToLanes = {}
    dicLaneToFoeLanes = {}
    dicLaneToLaneDir = {}  # {[(“lane1”,"lane2"),straight]}
    dicAllLaneToLanes= {}
    dicLeaderLink = {}  #用于存储换道时的前后车相对关系，比如Dao.dicLeaderLink[1]=2，1号车的前车是2号车，用完后需要删除
    init_finished = False

    def __init__(self):
        pass

    @classmethod
    def init_from_begin(cls, userdata):
        """每次脚本打开时必须运行此函数"""
        userdata["VehicleDictionary"] = {}
        userdata["TrajectoryDictionary"] = {}
        cls.vehicle_dictionary = userdata["VehicleDictionary"]
        cls.trajectory_dictionary = userdata["TrajectoryDictionary"]

        log.info("initial begin")
        cls.init_lane_shapes()
        cls.refresh_dao()
        cls.init_finished = True

    @classmethod
    def init_lane_shapes(cls):
        """初始化lane的数据"""
        # 以下几个函数顺序不能乱
        cls.initlaneshapes()
        cls.create_lane_object()
        # cls.init_internal_foe_lane()
        cls.init_next_lane()
        cls.init_lane_type()

    @classmethod
    def initlaneshapes(cls):
        g_dicLaneShapes = {}
        g_dicLaneFromLanes = {}
        g_dicLaneToLanes = {}
        g_dicLaneToFoeLanes = {}
        g_dicAllLaneToLanes = {}

        junctionList = PanoTrafficAPI.PanoTrafficApi_GetJunctionList()
        if len(junctionList) > 0:
            log.info("acquire Lane process start")
            for junctionId in junctionList:
                # log.info("Current loop junction: ", junctionId)
                incomingLanes = PanoTrafficAPI.PanoTrafficApi_GetIncomingLanes(junctionId)

                #E开头的道路计算道路中心线信息并放入g_dicLaneShapes
                for incLaneId in incomingLanes:
                    # log.info("Current loop junction: ", junctionId, "current loop incoming lane: ", incLaneId)
                    normalLaneShape = PanoSimTrafficAPI2.PanoAPI.myGetLaneShape(incLaneId)
                    if (len(normalLaneShape) > 1):
                        resizedLaneShape = AlgorithmInterface.getLaneShapeWithOnlyXY(normalLaneShape)
                        g_dicLaneShapes[incLaneId] = resizedLaneShape

                    else:
                        log.warning("Warning: [{}] has no point list from getLaneShape".format(incLaneId))

                # J开头的道路计算道路中心线信息并放入g_dicLaneShapes
                internalLanes = PanoTrafficAPI.PanoTrafficApi_GetInternalLanes(junctionId) # 注意这里，把小junction分隔开的道路也囊括进去了
                for intLaneId in internalLanes:
                    log.info("Current loop junction: {},current loop internal lane:{} ".format(junctionId,intLaneId))
                    normalLaneShape = PanoSimTrafficAPI2.PanoAPI.myGetLaneShape(intLaneId)
                    # log.info("Internal Lanes", intLaneId, normalLaneShape)
                    if (len(normalLaneShape) > 1):
                        resizedLaneShape = AlgorithmInterface.getLaneShapeWithOnlyXY(normalLaneShape)
                        g_dicLaneShapes[intLaneId] = resizedLaneShape

                    else:
                        log.warning("Warning: [{}] has no point list from getLaneShape".format(intLaneId))

                # https://segmentfault.com/a/1190000004457595?ref=myread 求线段交点
                """初始化FOE的lane,之所以没有和上面的for循环合并，是因为需要所有的内部道路的resizedLaneShape计算完毕"""
                for intLaneId in internalLanes:
                    cls.lane_dictionary[intLaneId] = InternalLane(intLaneId)  #实例化对象并放入lane_dictionary
                    foe_lanes_and_types = PanoSimTrafficAPI2.PanoAPI.myGetInternalFoeLanes(intLaneId)
                    #log.info('foe_lanes_and_types is ',foe_lanes_and_types)
                    if foe_lanes_and_types:
                        for foe_lane_and_type in foe_lanes_and_types:
                            flag = False
                            # foe_lane_and_type下标0是lane id，下标1是冲突类型的结构体
                            foe_lane_id = foe_lane_and_type[0]
                            try:
                                foe_type = foe_lane_and_type[1].value #这个是不同的平台返回的数据类型不一样，就如此处理了
                            except:
                                foe_type = foe_lane_and_type[1]
                            foe_lane_shape = g_dicLaneShapes[foe_lane_id]
                            current_lane_shape = g_dicLaneShapes[intLaneId]

                            #寻找交点
                            for current_index in range(len(current_lane_shape)-1):
                                for foe_index in range(len(foe_lane_shape)-1):
                                    start1 = [foe_lane_shape[foe_index][0], foe_lane_shape[foe_index][1]]  # foe的前一个顶点
                                    end1 = [foe_lane_shape[foe_index+1][0], foe_lane_shape[foe_index+1][1]]  # foe的后一个顶点
                                    start2 =[current_lane_shape[current_index][0], current_lane_shape[current_index][1]]  # 当前lane的前一个顶点
                                    end2=[current_lane_shape[current_index+1][0], current_lane_shape[current_index+1][1]]  # 当前lane的后一个顶点
                                    # log.info(start1, end1, start2, end2)
                                    result = AlgorithmInterface.intersection(start1, end1, start2, end2)
                                    # 如果result(X,Y)存在
                                    if result:
                                        def caculateStation(xa, ya, xb, yb):
                                            dy = yb - ya
                                            dx = xb - xa
                                            result_s = math.sqrt((dy ** 2) + (dx ** 2))
                                            return result_s

                                        flag = True
                                        x, y = result
                                        x1, y1 = start2

                                        s = caculateStation(x1, y1, x, y)+current_lane_shape[current_index][2]

                                        # 求向量夹角
                                        my_vector = np.array([end2[0] - start2[0], end2[1] - start2[1]])
                                        foe_vector = np.array([end1[0] - start1[0], end1[1] - start1[1]])

                                        module_x = np.linalg.norm(my_vector)
                                        module_y = np.linalg.norm(foe_vector)

                                        cos_angle = my_vector.dot(foe_vector) / (module_x * module_y)
                                        vectorial_angle = np.arccos(cos_angle)
                                        # 求向量夹角Done

                                        cls.lane_dictionary[intLaneId].foe_lane_type_points_s.append((foe_lane_id, foe_type, (x, y),s,vectorial_angle))

                                    if flag:  # 跳出里层循环
                                        break
                                if flag:  # 跳出外层循环
                                    break

                    # 按S排序
                    cls.lane_dictionary[intLaneId].foe_lane_type_points_s.sort(key=lambda item: item[3])


        else:
            log.info("必须在有路口的地图跑这个脚本")

        # lane to lanes. 这里是忽略中间车道，直接访问下一个车道，这里是获取lane to lane的结构
        for i in g_dicLaneShapes:
            toLanes = []
            dirs = PanoTrafficAPI.PanoTrafficApi_GetValidDirections(i)
            log.info("directions of {} is {}".format(i,dirs))
            # if (len(dirs) <= 0):
            #     continue  # internal lane用这个函数没用，因为没有direction
            for dir in dirs:
                toLanesAll = PanoTrafficAPI.PanoTrafficApi_GetNextLanes(i, dir)
                log.info("tolane of {} in {} is {}".format(i,dir,toLanes))
                if toLanesAll != None:
                    toLanes.append(toLanesAll)
                    # for toLane in toLanesAll:
                    #     toLanes.append(toLane)
            g_dicLaneToLanes[i] = toLanes

            #getAllNextLanes(lane, direction)->[[lane]]:
            all_to_lanes= {} #e.g. {'stright':['gneJ1_0_0','gneE1_0']}
            for dir in dirs:
                log.info("getAllNextLanes test")
                toLanes = PanoTrafficAPI.PanoTrafficApi_GetNextLanes(i, dir)
                log.info("tolane of {} in {} is {}".format(i,dir,toLanes))
                if toLanes != "":
                    all_to_lanes[dir]=toLanes
            g_dicAllLaneToLanes[i]=all_to_lanes
        # lane with to-foe lanes ,it seems no ues,下面这个g_dicLaneToFoeLanes是空的，没有用
        for i in g_dicLaneShapes:
            foeLanes = []
            isDone = False
            if (i not in g_dicLaneToLanes): continue
            lane1ToLanes = g_dicLaneToLanes[i]
            for j in g_dicLaneShapes:
                if i == j: continue
                if j not in g_dicLaneToLanes: continue  # 这里没有考虑非incoming lane
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
        cls.dicLaneShapes = g_dicLaneShapes
        cls.dicLaneFromLanes = g_dicLaneFromLanes
        cls.dicLaneToLanes = g_dicLaneToLanes
        cls.dicLaneToFoeLanes = g_dicLaneToFoeLanes
        cls.dicAllLaneToLanes= g_dicAllLaneToLanes
        return

    @classmethod
    def create_lane_object(cls):
        # LaneToLanes这个应该换成所有lane，并不是，internal_lane的实例化对象过程已在foeLane初始化的时候完成了
        for lane_id in cls.dicLaneToLanes.keys():
            if lane_id not in cls.lane_dictionary.keys():
                cls.lane_dictionary[lane_id] = Lane(lane_id)
            for lane_to_lanes in cls.dicLaneToLanes[lane_id]:
                for lane in lane_to_lanes:
                    if lane not in cls.lane_dictionary.keys(): #因为上文已经实例化了foe_lane，所以这里不会把internal_lane覆盖成lane了
                        cls.lane_dictionary[lane] = Lane(lane)

        #调试才用
        log.info("lane_dictionary")
        log.info("{}".format(cls.lane_dictionary.keys()))
        log.info("-----------------------------------------------------------------------")
        log.info("dicLaneToLanes")
        log.info("{}".format(cls.dicLaneToLanes))

    @classmethod
    def init_internal_foe_lane(cls):
        junctionList = PanoTrafficAPI.PanoTrafficApi_GetJunctionList()

        pass

    @classmethod
    def init_next_lane(cls):
        for lane_id in cls.dicLaneToLanes.keys():
            for lane_to_lanes in cls.dicLaneToLanes[lane_id]:
                for i in range(0, len(lane_to_lanes)):
                    if i == 0:
                        if lane_to_lanes[0] not in cls.lane_dictionary[lane_id].next_lane:
                            cls.lane_dictionary[lane_id].next_lane.append(lane_to_lanes[0])
                            direction=cls.get_direction_from_lane_to_lane(lane_id,lane_to_lanes[0])
                            cls.dicLaneToLaneDir[(lane_id,lane_to_lanes[0])]=direction

                            cls.lane_dictionary[lane_to_lanes[0]].previous_lane.append(lane_id)

                    else:
                        if lane_to_lanes[i] not in cls.lane_dictionary[lane_to_lanes[i - 1]].next_lane:
                            cls.lane_dictionary[lane_to_lanes[i - 1]].next_lane.append(lane_to_lanes[i])
                            # internal Lane 是不会有dirs的
                            direction = cls.get_direction_from_lane_to_lane(lane_to_lanes[i - 1], lane_to_lanes[0])
                            cls.dicLaneToLaneDir[(lane_to_lanes[i - 1], lane_to_lanes[0])] = direction

                            cls.lane_dictionary[lane_to_lanes[i]].previous_lane.append(lane_to_lanes[i - 1])


    @classmethod
    def init_lane_type(cls):
        junction_list = PanoTrafficAPI.PanoTrafficApi_GetJunctionList()
        for junction_id in junction_list:
            incoming_lanes = PanoTrafficAPI.PanoTrafficApi_GetIncomingLanes(junction_id)
            for item_id in incoming_lanes:
                if item_id in cls.lane_dictionary.keys() and cls.lane_dictionary[item_id].type==None: #==None是为了避免重复赋值，比如小junction会把上一条internal lane 作为incoming lane
                    cls.lane_dictionary[item_id].type = TypesOfRoad.NORMAL_LANE
                    if not PanoTrafficAPI.PanoTrafficApi_GetIsDeadEnd(item_id):
                        cls.lane_dictionary[item_id].set_is_dead_end(False)

            internal_lanes = PanoTrafficAPI.PanoTrafficApi_GetInternalLanes(junction_id)
            for int_lane_id in internal_lanes:
                if int_lane_id in cls.lane_dictionary.keys():
                    cls.lane_dictionary[int_lane_id].type = TypesOfRoad.INTERNAL_LANE
                    cls.lane_dictionary[int_lane_id].set_is_dead_end(False)

        for lane in Dao.lane_dictionary.values():
            if len(lane.next_lane) == 0:
                lane.type = TypesOfRoad.NORMAL_LANE
                lane.is_dead_end = True

    @classmethod
    def refresh_dao(cls):
        """更新Dao层数据"""
        vehicle_list=PanoTrafficAPI.PanoTrafficApi_GetVehicleList()

        if False and SimPlatformAPI.API_Name=='PanoAPI':

            vehicle_list_in_Dao=list(cls.vehicle_dictionary.keys())

            for old_vehicle_id in vehicle_list_in_Dao:
                if old_vehicle_id not in vehicle_list:
                    del cls.vehicle_dictionary[old_vehicle_id]
                    del cls.trajectory_dictionary[old_vehicle_id]

            vehicle_list_in_Dao = list(cls.vehicle_dictionary.keys())
            max_num_elements = 50
            to_ctrl_num = max_num_elements - cls.vehicle_dictionary.__len__()

            if to_ctrl_num>5: #少了5辆车才考虑重新添加
                vehicle_list = vehicle_list_in_Dao+(random.sample(vehicle_list, min(len(vehicle_list), to_ctrl_num)))
            else:
                vehicle_list= vehicle_list_in_Dao #如果有车被SUMO删去了，这个是有问题的
        #转化为set，为了之后点集pop出数据时，能更快地完成,且set中没有重复的值
        vehicle_set = set(vehicle_list)

        #这是为了在主车周围的车才行进计算
        if vehicle_list[0]==0 and vehicle_list.__len__()>0:
            ego_x=PanoTrafficAPI.PanoTrafficApi_GetVehicleX(0)
            ego_y=PanoTrafficAPI.PanoTrafficApi_GetVehicleY(0)
            for vehId in vehicle_list:
                x=PanoTrafficAPI.PanoTrafficApi_GetVehicleX(vehId)
                y=PanoTrafficAPI.PanoTrafficApi_GetVehicleY(vehId)

                if (x-ego_x)**2+(y-ego_y)**2>40000: #40000开方200，节约计算量
                    vehicle_set.remove(vehId)

            del vehicle_list[0]
            vehicle_set.remove(0)

        for old_vehicle_id in set(cls.vehicle_dictionary.keys()):
            if old_vehicle_id not in vehicle_set:
                del cls.vehicle_dictionary[old_vehicle_id]
                del cls.trajectory_dictionary[old_vehicle_id]

        for vehicle_id in vehicle_set:
            if vehicle_id !=0 and vehicle_id not in set(cls.vehicle_dictionary.keys()): #0号车是主车
                cls.vehicle_dictionary[vehicle_id] = Vehicle(vehicle_id)
                cls.trajectory_dictionary[vehicle_id] = Trajectory(vehicle_id)

        # 调试才用
        # log.info("vehicles are " + str(cls.vehicle_dictionary.keys()))

    # 目前似乎没什么用
    @classmethod
    def update_userdata(cls, userdata):
        userdata["VehicleDictionary"] = cls.vehicle_dictionary
        userdata["TrajectoryDictionary"] = cls.trajectory_dictionary
        return

    @classmethod
    def get_vehicle_by_id(cls, car_id):
        if car_id is None:
            return None
        try:
            vehicle = cls.vehicle_dictionary[car_id]
        except:
            log.info("{} is not in vehicle_dictionary".format(car_id))
            return None
        else:
            return vehicle

    @classmethod
    def get_trajectory_by_id(cls, car_id):
        return cls.trajectory_dictionary[car_id]

    # 函数: get_direction_from_lane_to_lane
    # 用途: 获取lane1到lane2是什么方向
    # [i]: int,int - lane1id,lane2id - 道路的id
    # [o]: int - direction 左，直行，右
    @classmethod  # 声明为抽象方法，子类必须重写的方法
    def get_direction_from_lane_to_lane(cls, current_lane_id, next_lane_id):

        result_direction=None
        #internal Lane 是不会有dirs的
        dirs = PanoTrafficAPI.PanoTrafficApi_GetValidDirections(current_lane_id)
        if (len(dirs) <= 0):
            log.info("[{}] has no directions".format(current_lane_id))
        for dir in dirs:
            toLanes_list = PanoTrafficAPI.PanoTrafficApi_GetNextLanes(current_lane_id, dir)

            for toLanes in toLanes_list:
                if next_lane_id in toLanes:
                    result_direction = dir


        return result_direction

    # 函数: insert_vehicle_to_dict
    # 用途: 添加车辆实例和轨迹实例到Dao层
    # [i]: int - veh_id 车辆的id
    # [o]: NULL
    @classmethod
    def insert_vehicle_to_dict(cls,veh_id):
        cls.vehicle_dictionary[veh_id] = Vehicle(veh_id)
        cls.trajectory_dictionary[veh_id] = Trajectory(veh_id)

    # 函数: refresh_parameters
    # 用途: 获取lane1到lane2是什么方向
    # [i]: int - veh_id 车辆的id
    # [o]: NULL
    @classmethod #TODO 该删除就删除
    def refresh_parameters(cls):
        cls.vehicle_dictionary[veh_id] = Vehicle(veh_id)
        cls.trajectory_dictionary[veh_id] = Trajectory(veh_id)

    # 函数: get_vehicle_object
    # 用途: 获取车辆实例
    # [i]: int - veh_id 车辆的id
    # [o]: Vehicle - veh_ob 车辆的实例
    @classmethod
    def get_vehicle_object(cls, veh_id):
        try:
            veh_ob=Dao.vehicle_dictionary[veh_id]
        except KeyError:
            # cls.insert_vehicle_to_dict(veh_id)
            # return cls.get_vehicle_object(veh_id)
            return None
        else:
            return veh_ob