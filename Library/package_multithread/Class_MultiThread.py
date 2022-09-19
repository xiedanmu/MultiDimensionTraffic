# import random
from Library.package_multithread import module_name as DLL
from concurrent.futures import ThreadPoolExecutor

import logging
log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())

# # max_workers=None 表示最大线程数是本机CPU物理核数
# with ThreadPoolExecutor(max_workers=None) as ex:
#     results = ex.map(lambda x: DLL.some_fn_python_name(x, random.uniform(0, 10)), [1, 2, 3])
#     for value in results:
#         print(value)

class MultiThreadPool:
    ex = ThreadPoolExecutor(max_workers=None)# max_workers=None 表示最大线程数是本机CPU物理核数
    def __init__(self):
        pass
    def ThreadPoolExecutor(self):
        pass


    @classmethod
    def calculate_track(cls,
                        trajectory,
                        connecting_flag,
                        vehicle_length,
                        cur_lane_id,
                        cur_lane_max_s,
                        cur_lane_is_dead_end,
                        distance_from_lane_start,
                        continuous_laneid,
                        continuous_length,
                        continuous_shape,
                        current_continuous_laneid):
        """多线程计算IDM模型"""

        def get_result(thread_result):
            # 得到结果了
            try:
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set.extend(thread_result.result())
                trajectory.continuous_lane_id = current_continuous_laneid
                # print(current_continuous_laneid)
            except Exception as e:
                log.critical("thread_result x_y_laneid_s_l_cums_cuml_yaw_set is empty,and Exception are:{}".format(e))
                trajectory.calculate_done = True
            else:
                for item in thread_result.result():
                    trajectory.s_in_current_trajectory.append(item[5])  # 这里的5代表x_y_laneid_s_l_cums_cuml中cums的下标
                log.info("{} going to change calculate_done to true".format(trajectory.VehID))
                trajectory.calculate_done=True
                log.info("{} trajectory calculate_done is: {}".format(trajectory.VehID,trajectory.calculate_done))
                log.info("{} had finished the track caculation,and len of points is:{}".format(trajectory.VehID,len(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set)))

        args = [trajectory.forecast_time_range,
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set,
                connecting_flag,
                vehicle_length,
                cur_lane_id,
                cur_lane_max_s,
                cur_lane_is_dead_end,
                distance_from_lane_start,
                continuous_laneid,
                continuous_length,
                continuous_shape]

        # trajectory.continuous_lane_id.clear()

        thread_result = cls.ex.submit(lambda p: DLL.generate_point_set_track(*p), args)
        #get_result(thread_result)
        thread_result.add_done_callback(get_result)

    @classmethod
    def calculate_way_point(cls,
                            trajectory,
                            s_from_lane_start,
                            lane_width,
                            distance,
                            direction,
                            continuous_laneid,
                            continuous_length,
                            continuous_shape):
        """多线程计算换道模型（多项式）"""
        def get_result(thread_result):
            # 得到结果了
            try:
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set = thread_result.result()
                trajectory.continuous_lane_id = continuous_laneid['Middle'] + continuous_laneid['Left'] + continuous_laneid['Right']
            except Exception as e:
                log.critical("thread_result x_y_laneid_s_l_cums_cuml_yaw_set is empty,and Exception are:{}".format(e))
                trajectory.calculate_done = True
            else:
                trajectory.s_in_current_trajectory = []
                for item in trajectory.x_y_laneid_s_l_cums_cuml_yaw_set:
                    trajectory.s_in_current_trajectory.append(item[5])  # 这里的5代表x_y_laneid_s_l_cums_cuml中cums的下标
                log.info("{} going to change calculate_done to true".format(trajectory.VehID))
                trajectory.calculate_done=True
                log.info("{} trajectory calculate_done is: {}".format(trajectory.VehID, trajectory.calculate_done))
                log.info("{} had finished the track caculation,and len of points is:{}".format(trajectory.VehID,len(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set)))

        args = [s_from_lane_start,
                lane_width,
                distance,
                direction,
                continuous_laneid,
                continuous_length,
                continuous_shape]


        # for item in continuous_laneid['Middle']:
        #     trajectory.continuous_lane_id.append(item)
        # for item in continuous_laneid['Left']:
        #     trajectory.continuous_lane_id.append(item)
        # for item in continuous_laneid['Right']:
        #     trajectory.continuous_lane_id.append(item)
        # trajectory.continuous_lane_id.append(continuous_laneid['Middle'])
        # trajectory.continuous_lane_id.append(continuous_laneid['Left'])
        # trajectory.continuous_lane_id.append(continuous_laneid['Right'])

        # trajectory.continuous_lane_id.clear()

        thread_result = cls.ex.submit(lambda p: DLL.generate_point_set_waypoint(*p), args)
        #get_result(thread_result)
        thread_result.add_done_callback(get_result)
