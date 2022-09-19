# 线程返回的轨迹类
# [i] vehID
# [o] 需要设置预计的轨迹 [ [x1,y1,yaw1,speed1,t1], ..., [xn,yn,yawn,speedn,tn] ]
# [o] m_bTrajectoryAvailable 轨迹是否设置成功？ 最好在线程的最后赋值，不再做互锁了
# [o] m_dOldEnoughTime 可以是动态的值，返回这段轨迹可以在什么时候再次控制
#                      主线程会在这个时间后，再次调用线程开始决策
# import random
# random.seed(1)
import numpy as np
rng = np.random.Generator(np.random.MT19937(1))
class Trajectory:
    def __init__(self, car_id):
        self.VehID = car_id
        self.forecast_time_range=rng.integers(5, 9) # 预测未来2秒的路径
        self.x_y_laneid_s_l_cums_cuml_yaw_set=[] # list
        self.s_in_current_trajectory = []
        self.change_lane_needed_time=3+rng.integers(0, 3)
        self.calculate_done=True
        self.suspend_time=0
        self.continuous_lane_id=[]

    def delete_trajectory_data(self):
        self.x_y_laneid_s_l_cums_cuml_yaw_set = []  # list
        self.s_in_current_trajectory = []

    def set_forecast_time(self,forecast_time):
        self.forecast_time_range = forecast_time
