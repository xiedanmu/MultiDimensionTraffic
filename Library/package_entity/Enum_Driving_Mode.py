from enum import Enum


class DrivingMode(Enum):
    FOLLOW_DRIVING = 1
    LEFT_CHANGING_LANE = 2
    RIGHT_CHANGING_LANE = 3
    BREAKING_CHANGE_LANE = 4
    PASSING_INTERSECTION = 5
    AVOIDING = 6
