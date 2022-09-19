class Lane:
    """道路类"""
    # todo 目前只有junction的来源车道和内部车道会在Dao的Lane dictionary里面有数据
    def __init__(self, lane_id):
        self.id = lane_id
        self.type=None # Normal or Internal
        self.is_dead_end = None #Ture or False
        self.next_lane = []
        self.previous_lane = []

    def set_is_dead_end(self,true_or_false):
        self.is_dead_end = true_or_false


