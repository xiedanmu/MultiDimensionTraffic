from abc import ABCMeta, abstractmethod  # 实现接口的模块


class TrafficInterface(metaclass=ABCMeta):
    """
    计算交通层面的接口
    TrafficInterface的层级比MotivateInterface低，即不能在这个文件中导入 MotivateInterface
    """
    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def set_car_zone(vehicle):
        return

    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def two_car_distance(car1_id, car2_id):
        return

    # 函数: check_lane_exist
    # 用途: 判断车辆是否存在左车道或右车道
    # [i]: vehID - 车辆ID
    # [i]: dir -   Direction.LEFT：左侧；  Direction.RIGHT右侧；
    # [o]: Bool - True有车道，False无车道
    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def check_lane_exist(vehID, dir):
        return

    # 函数: check_free_change_lane
    # 用途: 判断车辆向左自由换道和向右自由换道是否满足条件
    # [i]: vehID - 车辆ID
    #    计算过程中ABCD四个条件都能满足才行
    # [o]: Direction.LEFT：左侧；  Direction.RIGHT右侧；NONE
    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def get_free_change_lane_direction(cls, vehID):
        return

    # 函数: get_lane_length
    # 用途: 获取道路长度
    # [i]: vehID - 车辆ID
    # [o]:
    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def get_lane_length(Lane_id):
        return

    # 函数:  get_conflict_vehid_in_foelane_by_vehid
    # 用途: 获取冲突车道的所有车辆
    # [i]: int - vehID - 车辆ID
    # [o]: List - foe_veh_list 冲突车道上的所有车
    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def get_conflict_vehid_in_foelane_by_vehid(vehID):
        return

    # 函数:  update_the_vehicle_area
    # 用途: 更新车辆的所在区域
    # [i]: Class_Vehicle - veh - 车辆对象实例
    # [o]: Bool - True Or False 是否成功
    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def update_the_vehicle_area(current_lane, vehicle):
        return

    # 函数: get_dirction_from_lane_to_lane
    # 用途: 获取lane1到lane2是什么方向
    # [i]: int,int - lane1id,lane2id - 道路的id
    # [o]: int - direction 左，直行，右
    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def get_direction_from_lane_to_lane(current_lane_id, next_lane_id):
        return

    # 函数: get_leader_vehicle
    # 用途: 获取leader vehicle
    # [i]: int - carID 车辆ID
    # [o]: int - leader_carID
    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def get_leader_vehicle(vehicle,trajectory):
        return

    # 函数: get_next_lane
    # 用途: 获取leader vehicle
    # [i]: int - carID 车辆ID
    # [o]: int - leader_carID
    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def get_next_lane_by_trajectory(trajectory):

        return

    # 函数: update_vehicle_num_in_world
    # 用途: 在地图上生成汽车
    # [i]: int - carID 车辆ID
    # [o]: int - leader_carID
    @abstractmethod
    def update_vehicle_num_in_world(trajectory):

        raise NotImplementedError

    # 函数: vehicle_generation
    # 用途: 在地图上生成汽车
    # [i]: int - car_num 汽车数量
    # [o]: None
    # [参数解释]
    # generateMode = 0, 0 - -开放性边界条件（消失的车和补充的无关），1 - -周期性边界条件（消失的车辆会产生重新从开头进入路网）
    # initial Density = 0, 初始全路网（不含内部车道）的车辆密度，需要转化为车辆数目：（密度 * 道路长度 /（车辆长度 + 2）后取整，均匀分布到正常车道）
    # arrivePossible = 0，对应于开放性边界条件下车辆进入路网的可能性，
    @abstractmethod
    def vehicle_generation(self,generateMode = 0, initial_density = 0, arrivePossible = 0):

        raise NotImplementedError