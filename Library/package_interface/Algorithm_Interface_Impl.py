import copy
import math
from typing import List
from bisect import bisect_left
from Library.package_interface.Algorithm_Interface import AlgorithmInterface
import logging
log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())

class AlgorithmInterfaceImpl(AlgorithmInterface):
    @staticmethod
    def take_closest(myList, myNumber):
        """
        Assumes myList is sorted. Returns closest value to myNumber.

        If two numbers are equally close, return the smallest number.
        """
        pos = bisect_left(myList, myNumber)
        if pos == 0:
            return myList[0]
        if pos == len(myList):
            return myList[-1]
        before = myList[pos - 1]
        after = myList[pos]
        if after - myNumber < myNumber - before:
            return after
        else:
            return before

    # 函数: getLaneShapeWithOnlyXY
    # 用途: 根据x,y坐标获取目标车道的x,y,s,yaw坐标  yaw角与GPS保持一致
    # [i]: {[x,y], ..., [x,y]} 传入目标车道道路中间线的信息
    # [o]: [[x,y,s,yaw], ..., [x,y,s,yaw]] 传入目标车道道路中间线的信息
    @staticmethod
    def getLaneShapeWithOnlyXY(points):
        points_temp = copy.deepcopy(points)

        def caculate2pointsYaw(x1, y1, x2, y2):
            yaw = 0
            dy = y2 - y1
            dx = x2 - x1
            if dx == 0 and dy > 0:
                yaw = 0
            if dx == 0 and dy < 0:
                yaw = 180
            if dy == 0 and dx > 0:
                yaw = 90
            if dy == 0 and dx < 0:
                yaw = 270
            if dx > 0 and dy > 0:
                yaw = math.atan(dx / dy) * 180 / math.pi
            elif dx < 0 and dy > 0:
                yaw = 360 + math.atan(dx / dy) * 180 / math.pi
            elif dx < 0 and dy < 0:
                yaw = 180 + math.atan(dx / dy) * 180 / math.pi
            elif dx > 0 and dy < 0:
                yaw = 180 + math.atan(dx / dy) * 180 / math.pi

            return yaw * math.pi / 180

        def caculateStation(x1, y1, x2, y2):
            s = 0
            dy = y2 - y1
            dx = x2 - x1
            s = math.sqrt((dy ** 2) + (dx ** 2))

            return s

        lenLane = len(points_temp)
        i = 0

        if i == 0:
            S_tuple = (0.)
            Yaw_tuple = (
                caculate2pointsYaw(points_temp[i][0], points_temp[i][1], points_temp[i + 1][0], points_temp[i + 1][1]))
            addedTuple = (S_tuple, Yaw_tuple)
            points_temp[i] = points_temp[i] + addedTuple
            i = i + 1

        while i < lenLane - 1:
            deltaS = caculateStation(points_temp[i - 1][0], points_temp[i - 1][1], points_temp[i][0], points_temp[i][1])
            Yaw = caculate2pointsYaw(points_temp[i][0], points_temp[i][1], points_temp[i + 1][0], points_temp[i + 1][1])
            addedS = points_temp[i - 1][2] + deltaS
            addTuple = (addedS, Yaw)
            points_temp[i] = points_temp[i] + addTuple
            i = i + 1

        if i == lenLane - 1:  # 倒数第一个点与倒数第二个点保持一致
            deltaS = caculateStation(points_temp[i - 1][0], points_temp[i - 1][1], points_temp[i][0], points_temp[i][1])
            Yaw = caculate2pointsYaw(points_temp[i - 1][0], points_temp[i - 1][1], points_temp[i][0], points_temp[i][1])
            addedS = points_temp[i - 1][2] + deltaS
            addTuple = (addedS, Yaw)
            points_temp[i] = points_temp[i] + addTuple

        return points_temp

    @staticmethod
    def get_waypoint(s, D, W):
        """多项式航路点模型,返回SL坐标系上的L"""
        point_x = s
        if D < 0.01:
            D = 0.01
        point_y = W * (6 * math.pow(point_x / D, 5) - 15 * math.pow(point_x / D, 4) + 10 * math.pow(point_x / D, 3))
        return point_y

    # 函数: get_yaw_in_generating_points
    # 用途: 根据x,y坐标点集获取对应的yaw角度
    # [i]: {[x,y,cumS,cumL], ..., [x,y,cumS,cumL]} 传入车辆所在位置的点集，sl|| x_y_laneid_s_l_cums_cuml
    # [o]: {[x,y,cumS,cumL,yaw], ..., [x,y,cumS,cumL,yaw]}传出车辆所在位置的点集和sl+角度||x_y_laneid_s_l_cums_cuml_yaw
    @staticmethod
    def get_yaw_in_generating_points(points):
        points_temp = copy.deepcopy(points)

        def caculate2pointsYaw(x1, y1, x2, y2):
            yaw = 0
            dy = y2 - y1
            dx = x2 - x1
            if dx == 0 and dy > 0:
                yaw = 0
            if dx == 0 and dy < 0:
                yaw = 180
            if dy == 0 and dx > 0:
                yaw = 90
            if dy == 0 and dx < 0:
                yaw = 270
            if dx > 0 and dy > 0:
                yaw = math.atan(dx / dy) * 180 / math.pi
            elif dx < 0 and dy > 0:
                yaw = 360 + math.atan(dx / dy) * 180 / math.pi
            elif dx < 0 and dy < 0:
                yaw = 180 + math.atan(dx / dy) * 180 / math.pi
            elif dx > 0 and dy < 0:
                yaw = 180 + math.atan(dx / dy) * 180 / math.pi

            return yaw * math.pi / 180

        lenLane = len(points_temp)
        i = 0

        if i == 0:  # todo 和while i < lenLane - 1重复了
            Yaw = caculate2pointsYaw(points_temp[i][0], points_temp[i][1], points_temp[i + 1][0], points_temp[i + 1][1])
            points_temp[i] = points_temp[i] + (Yaw,)
            i = i + 1

        while i < lenLane - 1:
            Yaw = caculate2pointsYaw(points_temp[i][0], points_temp[i][1], points_temp[i + 1][0], points_temp[i + 1][1])
            points_temp[i] = points_temp[i] + (Yaw,)
            i = i + 1

        if i == lenLane - 1:  # 倒数第一个点与倒数第二个点保持一致
            Yaw = caculate2pointsYaw(points_temp[i - 1][0], points_temp[i - 1][1], points_temp[i][0], points_temp[i][1])
            points_temp[i] = points_temp[i] + (Yaw,)

        return points_temp

    # 函数: get_tuple_by_linear_interpolation
    # 用途: 根据s值用线性插值计算轨迹实际的位置
    # [i]: {[x, y, laneid, s, l, cums, cuml, yaw], ..., [x, y, laneid, s, l, cums, cuml, yaw]},s 传入车辆轨迹的list和实际s
    # [o]: (x, y, laneid, s, l, cums, cuml, yaw),pos传出线性插值后的数据,以及较小的那个索引
    @classmethod
    def get_tuple_by_linear_interpolation(cls, trajectory, s):
        pos = bisect_left(trajectory.s_in_current_trajectory, s)-1  # 较小的那个索引
        #车辆速度为0就会导致pos==-1
        if pos ==-1:
            pos=0
        if pos == (len(trajectory.s_in_current_trajectory) - 1):
            log.info('Need to calculate new trajectory RIGHT NOW!')
            return trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos], pos
        elif pos == len(trajectory.s_in_current_trajectory):
            log.info('real s is {} the max s in trajectory is {}'.format(s,trajectory.s_in_current_trajectory[pos - 1]))
            log.info('trajectory is not enough!,then weird questions will happen irretrievably,PLS check the logic')
            return None
        delta = s - trajectory.s_in_current_trajectory[pos]
        interval = trajectory.s_in_current_trajectory[pos + 1] - trajectory.s_in_current_trajectory[pos]
        if interval == 0:
            return trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos], pos
        ratio = delta / interval
        real_x = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][0] + ratio * (
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos + 1][0] -
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][0])
        real_y = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][1] + ratio * (
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos + 1][1] -
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][1])
        lane_id = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][2]
        real_s = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][3] + ratio * (
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos + 1][3] -
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][3])
        real_l = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][4] + ratio * (
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos + 1][4] -
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][4])
        real_cums = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][5] + ratio * (
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos + 1][5] -
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][5])
        real_cuml = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][6] + ratio * (
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos + 1][6] -
                trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][6])

        '''yaw角从2Π跳到0或者从0跳到2Π会出现转圈的现象，需要做一个判断'''
        if abs(trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos + 1][7]-trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][7])>3.14/2:
            real_yaw=trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos + 1][7]
        else:
            real_yaw = trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][7] + ratio * (
                    trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos + 1][7] -
                    trajectory.x_y_laneid_s_l_cums_cuml_yaw_set[pos][7])
        return (real_x, real_y, lane_id, real_s, real_l, real_cums, real_cuml, real_yaw), pos

    # 给定两条线段（表示为起点start = {X1, Y1}和终点end = {X2, Y2}），如果它们有交点，请计算其交点，没有交点则返回空值。
    # 要求浮点型误差不超过10 ^ -6。若有多个交点（线段重叠）则返回X值最小的点，X坐标相同则返回Y值最小的点。
    @classmethod
    def intersection(cls, start1: List[float], end1: List[float], start2: List[float], end2: List[float]) -> List[
        float]:
        # 判断 (xk, yk) 是否在「线段」(x1, y1)~(x2, y2) 上
        # 这里的前提是 (xk, yk) 一定在「直线」(x1, y1)~(x2, y2) 上
        def inside(x1, y1, x2, y2, xk, yk):
            # 若与 x 轴平行，只需要判断 x 的部分
            # 若与 y 轴平行，只需要判断 y 的部分
            # 若为普通线段，则都要判断
            return (x1 == x2 or min(x1, x2) <= xk <= max(x1, x2)) and (y1 == y2 or min(y1, y2) <= yk <= max(y1, y2))

        def update(ans, xk, yk):
            # 将一个交点与当前 ans 中的结果进行比较
            # 若更优则替换
            return [xk, yk] if not ans or [xk, yk] < ans else ans

        x1, y1 = start1
        x2, y2 = end1
        x3, y3 = start2
        x4, y4 = end2

        # 起点或终点有点重合的时候，乘除的时候得不到1.0，故做如下处理
        if x1 == x3 and y1 == y3:
            return [x1, y1]
        elif x2 == x3 and y2 == y3:
            return [x2, y2]
        elif x1 == x4 and y1 == y4:
            return [x1, y1]
        elif x2 == x4 and y2 == y4:
            return [x2, y2]

        ans = list()
        # 判断 (x1, y1)~(x2, y2) 和 (x3, y3)~(x4, y3) 是否平行
        if (y4 - y3) * (x2 - x1) == (y2 - y1) * (x4 - x3):
            # 若平行，则判断 (x3, y3) 是否在「直线」(x1, y1)~(x2, y2) 上
            if (y2 - y1) * (x3 - x1) == (y3 - y1) * (x2 - x1):
                # 判断 (x3, y3) 是否在「线段」(x1, y1)~(x2, y2) 上
                if inside(x1, y1, x2, y2, x3, y3):
                    ans = update(ans, x3, y3)
                # 判断 (x4, y4) 是否在「线段」(x1, y1)~(x2, y2) 上
                if inside(x1, y1, x2, y2, x4, y4):
                    ans = update(ans, x4, y4)
                # 判断 (x1, y1) 是否在「线段」(x3, y3)~(x4, y4) 上
                if inside(x3, y3, x4, y4, x1, y1):
                    ans = update(ans, x1, y1)
                # 判断 (x2, y2) 是否在「线段」(x3, y3)~(x4, y4) 上
                if inside(x3, y3, x4, y4, x2, y2):
                    ans = update(ans, x2, y2)
            # 在平行时，其余的所有情况都不会有交点
        else:
            # 联立方程得到 t1 和 t2 的值
            t1 = (x3 * (y4 - y3) + y1 * (x4 - x3) - y3 * (x4 - x3) - x1 * (y4 - y3)) / (
                    (x2 - x1) * (y4 - y3) - (x4 - x3) * (y2 - y1))
            t2 = (x1 * (y2 - y1) + y3 * (x2 - x1) - y1 * (x2 - x1) - x3 * (y2 - y1)) / (
                    (x4 - x3) * (y2 - y1) - (x2 - x1) * (y4 - y3))
            # 判断 t1 和 t2 是否均在 [0, 1] 之间
            if 0.0 <= t1 <= 1.0 and 0.0 <= t2 <= 1.0:
                ans = [x1 + t1 * (x2 - x1), y1 + t1 * (y2 - y1)]

        return ans
