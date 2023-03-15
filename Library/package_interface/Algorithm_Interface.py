from abc import ABCMeta, abstractmethod  # 实现接口的模块

from typing import List


class AlgorithmInterface(metaclass=ABCMeta):

    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def take_closest(myList, myNumber):
        return

    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def getLaneShapeWithOnlyXY(points):
        return

    @abstractmethod  # 声明为抽象方法，子类必须重写的方法
    def get_waypoint(s, D, W):
        return

    # 函数: get_yaw_in_generating_points
    # 用途: 根据x,y坐标点集获取对应的yaw角度
    # [i]: {[x,y,cumS,cumL], ..., [x,y,cumS,cumL]} 传入车辆所在位置的点集，sl
    # [o]: {[x,y,cumS,cumL,yaw], ..., [x,y,cumS,cumL,yaw]}传出车辆所在位置的点集和sl+角度
    @staticmethod
    def get_yaw_in_generating_points(points):
        return

    # 函数: get_tuple_by_linear_interpolation
    # 用途: 根据s值用线性插值计算轨迹实际的位置
    # [i]: {[x, y, laneid, s, l, cums, cuml, yaw], ..., [x, y, laneid, s, l, cums, cuml, yaw]},s 传入车辆轨迹的list和实际s
    # [o]: (x, y, laneid, s, l, cums, cuml, yaw),pos传出线性插值后的数据,以及较小的那个索引
    @classmethod
    def get_tuple_by_linear_interpolation(cls, trajectory, s,continue_lane_length):
        return

    # 给定两条线段（表示为起点start = {X1, Y1}和终点end = {X2, Y2}），如果它们有交点，请计算其交点，没有交点则返回空值。
    # 要求浮点型误差不超过10 ^ -6。若有多个交点（线段重叠）则返回X值最小的点，X坐标相同则返回Y值最小的点。
    @classmethod
    def intersection(cls, start1: List[float], end1: List[float], start2: List[float], end2: List[float]) -> List[
        float]:
        return