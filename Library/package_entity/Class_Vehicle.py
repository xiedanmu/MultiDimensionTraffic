# import random
# random.seed(1)
import numpy as np
rng = np.random.Generator(np.random.MT19937(1))
from Library.package_platformAPI import PanoSimTrafficAPI2
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
        self.vehicle_type = PanoSimTrafficAPI2.myGetVehicleType(car_id).name
        self.is_calculate_trajectory=False
        self.max_v = PanoSimTrafficAPI2.myGetVehicleMaxSpeed(car_id) + rng.integers(-20, 20)
        self.max_accel = PanoSimTrafficAPI2.myGetVehicleMaxAccel(car_id) + rng.integers(-1, 1)
        self.max_decel = PanoSimTrafficAPI2.myGetVehicleMaxDecel(car_id)
        self.current_speed = PanoSimTrafficAPI2.myGetVehicleSpeed(car_id)
        self.current_lane = PanoSimTrafficAPI2.myGetVehicleLane(car_id)  # 后续会变成一个对象,但是要确保体系中的lane包括了了PanoSim中的所有lane，有待验证
        self.current_acceleration = PanoSimTrafficAPI2.myGetVehicleAccel(car_id)
        self.current_X = PanoSimTrafficAPI2.myGetVehicleX(car_id)
        self.current_Y = PanoSimTrafficAPI2.myGetVehicleY(car_id)
        self.current_Yaw = PanoSimTrafficAPI2.myGetVehicleYaw(car_id)
        self.current_Route = PanoSimTrafficAPI2.myGetVehicleRoute(car_id)
        self.driving_mode = None  # Driving_Mode
        self.driving_mode_last_time_changed = True
        self.lane_type = TypesOfRoad.NORMAL_LANE
        self.located_area = LocatedArea.NORMAL
        self.is_xy_v_updated_lastT=True
        self.random_coefficients=rng.integers(0,10)

        self.going_to_update_acc=None
        self.going_to_update_speed=None
        self.going_to_update_x=None
        self.going_to_update_y=None
        self.going_to_update_yaw=None

    def set_speed(self, speed):
        self.current_speed = speed

    def set_acceleration(self, acceleration):
        self.current_acceleration = acceleration;

    def set_driving_mode(self, driving_mode):
        self.driving_mode = driving_mode
