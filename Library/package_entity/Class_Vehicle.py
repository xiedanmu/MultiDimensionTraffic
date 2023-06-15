# import random
# random.seed(1)
import numpy as np
rng = np.random.Generator(np.random.MT19937(1))
from Library.package_platformAPI.SimPlatformAPI import SimPlatformAPI
from Library.package_entity.Class_Trajectory import Trajectory
from Library.package_entity.Enum_Located_Area import LocatedArea
from Library.package_entity.Enum_Types_Of_Road import TypesOfRoad


# 定义一个车辆类
class Vehicle:
    """车辆类"""

    def __init__(self, car_id):
        self.Trajectory = Trajectory(car_id)
        self.id = car_id
        # Car = 0,Van = 1,Bus = 2,OtherVehicle = 3,Pedestrian = 10,NonMotorVehicle = 20,Others = 30
        self.vehicle_type = SimPlatformAPI.myGetVehicleType(car_id)
        self.is_calculate_trajectory=False
        self.max_v = SimPlatformAPI.myGetVehicleMaxSpeed(car_id) + rng.integers(-20, 20)
        #TODO 每10ms的加速度存储集，在后面第n个10ms用来计算，至于存多少个，是一个变量。有重叠的车直接停下来（第三套逻辑，暂时主干道且跟驰的情况下），等m秒后消失。shapely可用于检测点是否在一个矩形内

        self.a_dict= {i:0 for i in range(10)} #创建十个index且全为0的字典
        self.next_a_index=self.a_dict.__len__()-1 #下一步提取的a的index

        self.max_accel = SimPlatformAPI.myGetVehicleMaxAccel(car_id) #+ rng.integers(-1, 1)
        self.max_decel = SimPlatformAPI.myGetVehicleMaxDecel(car_id)
        self.current_speed = SimPlatformAPI.myGetVehicleSpeed(car_id)
        self.current_lane_id = SimPlatformAPI.myGetVehicleLane(car_id)  # 后续会变成一个对象,但是要确保体系中的lane包括了了PanoSim中的所有lane，有待验证
        self.current_acceleration = SimPlatformAPI.myGetVehicleAccel(car_id)
        self.current_X = SimPlatformAPI.myGetVehicleX(car_id)
        self.current_Y = SimPlatformAPI.myGetVehicleY(car_id)
        self.current_S = SimPlatformAPI.myGetDistanceFromLaneStart(car_id)
        self.current_Yaw = SimPlatformAPI.myGetVehicleYaw(car_id)
        self.current_Route = SimPlatformAPI.myGetVehicleRoute(car_id)
        self.driving_mode = None  # Driving_Mode
        self.driving_mode_last_time_changed = True
        self.lane_type = TypesOfRoad.NORMAL_LANE
        self.located_area = LocatedArea.NORMAL
        self.is_xy_v_updated_lastT=True
        self.random_coefficients=rng.integers(0,10)
        self.lane_is_changed= True
        self.length=SimPlatformAPI.myGetVehicleLength(car_id)
        self.width = SimPlatformAPI.getVehicleWidth(self.vehicle_type)
        self.ignored = False

        '''碰撞功能'''
        self.is_accidental = False
        self.accidental_time = 0
        self.going_to_delete = False

        self.going_to_update_acc=None
        self.going_to_update_speed=None
        self.going_to_update_x=None
        self.going_to_update_y=None
        self.going_to_update_yaw=None
        self.going_to_update_lane_id = None
        self.going_to_update_s = None
        self.going_to_update_l = None
        self.going_to_update_Route=None

        self.foe_vehicle=None
        self.following_model=None

        self.a_max = rng.uniform(3,6)  # 最大加速度
        self.v_desired = rng.integers(50,70)/ 3.6  # 汽车行驶期望速度 ,路口内要够慢
        self.delta = rng.integers(3,6)  # 自由加速指数δ
        self.t_desired = rng.uniform(1, 3) # 汽车安全车头时距T
        self.s0 = rng.uniform(1,3)  # 最小安全间距
        self.b_comfortable = rng.uniform(3, 6)  # 期望舒适减速度

        self.acc_used_account=0

    def set_speed(self, speed):
        self.current_speed = speed

    def set_acceleration(self, acceleration):
        self.current_acceleration = acceleration;

    def set_driving_mode(self, driving_mode):
        self.driving_mode = driving_mode
