from enum import Enum


class LocatedArea(Enum):
    ADJACENT_JUNCTION_AREA=1
    NORMAL=2
    INTERNAL=3
    FORCE_CHANGE_TO_LEFT=4
    FORCE_CHANGE_TO_RIGHT=5
