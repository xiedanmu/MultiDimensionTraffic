from abc import ABCMeta, abstractmethod  # 实现接口的模块


class MotivateInterface(metaclass=ABCMeta):

    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def get_and_update_accelerator(vehicle,trajectory) -> int:
        return 0

    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def get_and_update_accelerator(self) -> int:
        return

    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def check_change_lane(self):
        return

    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def change_to_left_lane(self):
        return

    @abstractmethod # 声明为抽象方法，子类必须重写的方法
    def generate_trajectory_point_set(self):
        return

    '''
    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def generate_point_set_idm(vehicle):
        return
    '''

    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def pop_point_in_trajectory(cls, vehicle, trajectory,delta_T):
        return

    @abstractmethod# 声明为抽象方法，子类必须重写的方法
    def generate_point_set_waypoint(self):
        return

    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def move_to(cls,vehID, x, y, yaw,lane,s,l):
        return