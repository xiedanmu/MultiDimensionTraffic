class UserParameters:
    def __init__(self):
        pass

    collision_stop=True
    @classmethod
    def enable_collision_stop(cls):
        cls.collision_stop = True

    @classmethod
    def disable_collision_stop(cls):
        cls.collision_stop = False
