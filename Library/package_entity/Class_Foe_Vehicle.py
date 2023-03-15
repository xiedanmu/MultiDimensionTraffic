import math

class FoeVehicle: #优先通行的敌对车及相关数据
    def __init__(self,
                 foe_car_id,
                 foe_s_to_entry_conflict_zone,
                 foe_s_to_leave_conflict_zone,
                 my_stop_s_at_entry_conflict_zone,
                 s_of_intersection_in_my_lane,
                 s_of_intersection_in_foe_lane,
                 alpha):

        self.my_stop_s_at_entry_conflict_zone = my_stop_s_at_entry_conflict_zone
        self.foe_s_to_leave_conflict_zone = foe_s_to_leave_conflict_zone
        self.foe_s_to_entry_conflict_zone = foe_s_to_entry_conflict_zone
        self.id = foe_car_id
        self.alpha = alpha
        self.s_of_intersection_in_my_lane = s_of_intersection_in_my_lane
        self.s_of_intersection_in_foe_lane = s_of_intersection_in_foe_lane

    @staticmethod
    def calculate_foe_entry_and_leave_data(alpha,intersection_s_of_my_lane,intersection_s_of_foe_lane,my_width,foe_width,foe_length):
        foe_s_to_entry_conflict_zone = intersection_s_of_foe_lane - (foe_width / 2 / math.tan(alpha) + my_width / 2 / math.sin(alpha))
        foe_s_to_leave_conflict_zone = intersection_s_of_foe_lane + (foe_width / 2 / math.tan(alpha) + my_width / 2 / math.sin(alpha))+foe_length
        my_stop_s_at_entry_conflict_zone = intersection_s_of_my_lane - (my_width / 2 / math.tan(alpha) + foe_width / 2 / math.sin(alpha))
        return foe_s_to_entry_conflict_zone,foe_s_to_leave_conflict_zone,my_stop_s_at_entry_conflict_zone

    def calculate_my_delta_s(self,my_s,foe_s,foe_length):

        if self.alpha > 0 and self.alpha <= math.pi / 2:
            if foe_s >= self.foe_s_to_entry_conflict_zone and foe_s <= self.foe_s_to_entry_conflict_zone + foe_length:
                '''IN CONFLICT ZONE with foe length'''
                return self.my_stop_s_at_entry_conflict_zone-my_s
            elif foe_s >= self.foe_s_to_entry_conflict_zone + foe_length and foe_s <= self.foe_s_to_leave_conflict_zone:
                '''IN CONFLICT ZONE without foe length'''
                return self.my_stop_s_at_entry_conflict_zone+(foe_s-foe_length-self.foe_s_to_entry_conflict_zone)*math.cos(self.alpha)-my_s
            # if foe_s >= self.foe_s_to_entry_conflict_zone and foe_s <= self.foe_s_to_leave_conflict_zone:
            #     '''IN CONFLICT ZONE'''
            #     return self.my_stop_s_at_entry_conflict_zone+(foe_s-self.foe_s_to_entry_conflict_zone)*math.cos(self.alpha)
            elif foe_s < self.foe_s_to_entry_conflict_zone:
                '''BEFORE CONFLICT ZONE'''
                rest_foe_s = self.s_of_intersection_in_foe_lane - foe_s
                foe_vehicle_map_to_my_lane_s = self.s_of_intersection_in_my_lane - rest_foe_s
                return foe_vehicle_map_to_my_lane_s-my_s
                #return self.my_stop_s_at_entry_conflict_zone - my_s
            elif foe_s > self.foe_s_to_leave_conflict_zone:
                '''AFTER CONFLICT ZONE'''
                return None
        elif self.alpha > math.pi / 2 and self.alpha <= math.pi:
            if foe_s >= self.foe_s_to_entry_conflict_zone and foe_s <= self.foe_s_to_leave_conflict_zone:
                '''IN CONFLICT ZONE'''
                return self.my_stop_s_at_entry_conflict_zone-my_s
            elif foe_s < self.foe_s_to_entry_conflict_zone:
                '''BEFORE CONFLICT ZONE'''
                rest_foe_s = self.s_of_intersection_in_foe_lane - foe_s
                foe_vehicle_map_to_my_lane_s = self.s_of_intersection_in_my_lane - rest_foe_s
                return foe_vehicle_map_to_my_lane_s-my_s
                #return self.my_stop_s_at_entry_conflict_zone - my_s
            elif foe_s > self.foe_s_to_leave_conflict_zone:
                '''AFTER CONFLICT ZONE'''
                return None







