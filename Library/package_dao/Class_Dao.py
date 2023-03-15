# 通过实体属性获取对象的Dao层 data access object
#
#
import math

import numpy as np

from Library.package_platformAPI.SimPlatformAPI import SimPlatformAPI
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
        cls.init_internal_foe_lane()
        cls.init_next_lane()
        cls.init_lane_type()

    @classmethod
    def initlaneshapes(cls):
        g_dicLaneShapes = {}
        g_dicLaneFromLanes = {}
        g_dicLaneToLanes = {}
        g_dicLaneToFoeLanes = {}
        g_dicAllLaneToLanes = {}

        junctionList = SimPlatformAPI.myGetJunctionList()
        if len(junctionList) > 0:
            log.info("acquire Lane process start")
            for junctionId in junctionList:
                # print("Current loop junction: ", junctionId)
                incomingLanes = SimPlatformAPI.myGetIncomingLanes(junctionId)

                #E开头的道路计算道路中心线信息并放入g_dicLaneShapes
                for incLaneId in incomingLanes:
                    # print("Current loop junction: ", junctionId, "current loop incoming lane: ", incLaneId)
                    normalLaneShape = SimPlatformAPI.myGetLaneShape(incLaneId)
                    if (len(normalLaneShape) > 1):
                        resizedLaneShape = AlgorithmInterface.getLaneShapeWithOnlyXY(normalLaneShape)
                        g_dicLaneShapes[incLaneId] = resizedLaneShape

                    else:
                        log.warning("Warning: [{}] has no point list from getLaneShape".format(incLaneId))

                # J开头的道路计算道路中心线信息并放入g_dicLaneShapes
                internalLanes = SimPlatformAPI.myGetInternalLanes(junctionId) # 注意这里，把小junction分隔开的道路也囊括进去了
                for intLaneId in internalLanes:
                    log.info("Current loop junction: {},current loop internal lane:{} ".format(junctionId,intLaneId))
                    normalLaneShape = SimPlatformAPI.myGetLaneShape(intLaneId)
                    # print("Internal Lanes", intLaneId, normalLaneShape)
                    if (len(normalLaneShape) > 1):
                        resizedLaneShape = AlgorithmInterface.getLaneShapeWithOnlyXY(normalLaneShape)
                        g_dicLaneShapes[intLaneId] = resizedLaneShape

                    else:
                        log.warning("Warning: [{}] has no point list from getLaneShape".format(intLaneId))

                # https://segmentfault.com/a/1190000004457595?ref=myread 求线段交点
                """初始化FOE的lane,之所以没有和上面的for循环合并，是因为需要所有的内部道路的resizedLaneShape计算完毕"""
                for intLaneId in internalLanes:
                    cls.lane_dictionary[intLaneId] = InternalLane(intLaneId)  #实例化对象并放入lane_dictionary
                    foe_lanes_and_types = SimPlatformAPI.myGetInternalFoeLanes(intLaneId)

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
                                    # print(start1, end1, start2, end2)
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
            dirs = SimPlatformAPI.myGetValidDirections(i)
            log.info("directions of {} is {}".format(i,dirs))
            # if (len(dirs) <= 0):
            #     continue  # internal lane用这个函数没用，因为没有direction
            for dir in dirs:
                toLanesAll = SimPlatformAPI.myGetAllNextLanes(i, dir)
                log.info("tolane of {} in {} is {}".format(i,dir,toLanes))
                if toLanesAll != None:
                    for toLane in toLanesAll:
                        toLanes.append(toLane)
            g_dicLaneToLanes[i] = toLanes

            #getAllNextLanes(lane, direction)->[[lane]]:
            all_to_lanes= {} #e.g. {'stright':['gneJ1_0_0','gneE1_0']}
            for dir in dirs:
                log.info("getAllNextLanes test")
                toLanes = SimPlatformAPI.myGetAllNextLanes(i, dir)
                log.info("tolane of {} in {} is {}".format(i,dir,toLanes))
                if toLanes != "":
                    all_to_lanes[dir]=toLanes
            g_dicAllLaneToLanes[i]=all_to_lanes
        # lane with to-foe lanes ,it seems no ues
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
        # print("Foe Lanes")
        # print(g_dicLaneFoeLanes)
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
        junctionList = SimPlatformAPI.myGetJunctionList()

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
        junction_list = SimPlatformAPI.myGetJunctionList()
        for junction_id in junction_list:
            incoming_lanes = SimPlatformAPI.myGetIncomingLanes(junction_id)
            for item_id in incoming_lanes:
                if item_id in cls.lane_dictionary.keys() and cls.lane_dictionary[item_id].type==None: #==None是为了避免重复赋值，比如小junction会把上一条internal lane 作为incoming lane
                    cls.lane_dictionary[item_id].type = TypesOfRoad.NORMAL_LANE
                    if not SimPlatformAPI.myIsDeadEnd(item_id):
                        cls.lane_dictionary[item_id].set_is_dead_end(False)

            internal_lanes = SimPlatformAPI.myGetInternalLanes(junction_id)
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
        vehicle_list=SimPlatformAPI.myGetVehicleList()
        vehicle_set = set(SimPlatformAPI.myGetVehicleList())

        if vehicle_list.__len__()>0 and vehicle_list[0]==0:
            ego_x=SimPlatformAPI.myGetVehicleX(0)
            ego_y=SimPlatformAPI.myGetVehicleY(0)
            for vehId in vehicle_list:
                x=SimPlatformAPI.myGetVehicleX(vehId)
                y=SimPlatformAPI.myGetVehicleY(vehId)

                if (x-ego_x)**2+(y-ego_y)**2>40000: #40000开方200，节约计算量
                    vehicle_set.remove(vehId)

            del vehicle_list[0]
            vehicle_set.remove(0)


        for old_vehicle_id in set(cls.vehicle_dictionary.keys()):
            if old_vehicle_id not in vehicle_set:
                del cls.vehicle_dictionary[old_vehicle_id]
                del cls.trajectory_dictionary[old_vehicle_id]

        for vehicle_id in vehicle_set:
            if vehicle_id not in set(cls.vehicle_dictionary.keys()):
                cls.vehicle_dictionary[vehicle_id] = Vehicle(vehicle_id)
                cls.trajectory_dictionary[vehicle_id] = Trajectory(vehicle_id)

        # 调试才用
        # print("vehicles are " + str(cls.vehicle_dictionary.keys()))

    # 目前似乎没什么用
    @classmethod
    def update_userdata(cls, userdata):
        userdata["VehicleDictionary"] = cls.vehicle_dictionary
        userdata["TrajectoryDictionary"] = cls.trajectory_dictionary
        return

    @classmethod
    def get_vehicle_by_id(cls, car_id):
        return cls.vehicle_dictionary[car_id]

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
        dirs = SimPlatformAPI.myGetValidDirections(current_lane_id)
        if (len(dirs) <= 0):
            log.info("[{}] has no directions".format(current_lane_id))
        for dir in dirs:
            toLanes_list = SimPlatformAPI.myGetAllNextLanes(current_lane_id, dir)

            for toLanes in toLanes_list:
                if next_lane_id in toLanes:
                    result_direction = dir


        return result_direction


