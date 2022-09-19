from Library.package_entity.Class_Lane import Lane


class InternalLane(Lane):
    """路口内部道路类"""

    # todo 目前只有junction的来源车道和内部车道会在Dao的Lane dictionary里面有数据
    def __init__(self, lane_id):
        Lane.__init__(self, lane_id)
        self.foe_lane_type_points_s = []  # {[lane1,冲突类型,(两条lane的交点),交点的s]} 例如[lane1,crossing,(10，15)，5]


# class FoeTypeAndConflictPoint:
#     def __init__(self, id):
#         self.id = id  # lane id
#         self.conflict_point  # 元组
