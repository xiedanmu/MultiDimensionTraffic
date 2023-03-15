class VehicleManager:

    vehicle_count=0
    Vehicle_license_plates_to_be_issued=1
    vehicle_dict=dict()

    @classmethod
    def initial(cls):
        pass

    @classmethod
    def get_vehicle_length_by_id(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]['length']

    @classmethod
    def get_speed(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]['v']

    @classmethod
    def get_vehicle_x(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]['x']

    @classmethod
    def get_vehicle_y(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]['y']

    @classmethod
    def get_vehicle_lane(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]['lane']

    @classmethod
    def get_distance_from_lane_start(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]['s']

    @classmethod
    def get_vehicle_list(cls):
        return list(cls.vehicle_dict.keys())

    @classmethod
    def get_vehicle_yaw(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]['yaw']

    @classmethod
    def get_route(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]['route']

    @classmethod
    def get_vehicle_accel(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]['a']

    @classmethod
    def change_speed(cls,vehicle_id, current_speed):
        cls.vehicle_dict[vehicle_id]['v']=current_speed

    @classmethod
    def change_route(cls,vehicle_id,route):
        cls.vehicle_dict[vehicle_id]['route'] = route

    @classmethod
    def get_vehicle_max_speed(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]["max_v"]

    @classmethod
    def get_vehicle_length(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]["length"]

    @classmethod
    def get_vehicle_max_acc(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]["max_acc"]

    @classmethod
    def get_vehicle_max_dec(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]["max_dec"]

    @classmethod
    def get_vehicle_lateral_offset(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]["l"]

    @classmethod
    def get_vehicle_type(cls,vehicle_id):
        return cls.vehicle_dict[vehicle_id]["type"]

    @classmethod
    def move_to(cls,vehicle_id,x, y, yaw,lane,s,l):
        cls.vehicle_dict[vehicle_id]["x"] = x
        cls.vehicle_dict[vehicle_id]["y"] = y
        cls.vehicle_dict[vehicle_id]["yaw"] = yaw
        cls.vehicle_dict[vehicle_id]["lane"] =lane
        cls.vehicle_dict[vehicle_id]["s"] = s
        cls.vehicle_dict[vehicle_id]["l"] = l


    @classmethod
    def delete_vehicle(cls,vehicle_id):
        del cls.vehicle_dict[vehicle_id]
        cls.vehicle_count-=1

    @classmethod
    def addVehicle(cls,x, y, current_s,lane_id, speed=0 ,type=0):
        id=cls.Vehicle_license_plates_to_be_issued
        cls.Vehicle_license_plates_to_be_issued += 1
        cls.vehicle_count += 1
        cls.vehicle_dict[id]=dict()
        #状态量
        cls.vehicle_dict[id]['x'] = x
        cls.vehicle_dict[id]['y'] = y
        cls.vehicle_dict[id]['yaw']=0
        cls.vehicle_dict[id]['s'] = current_s
        cls.vehicle_dict[id]['l'] = 0
        cls.vehicle_dict[id]['lane'] = lane_id
        cls.vehicle_dict[id]['v'] = speed
        cls.vehicle_dict[id]['a'] = 0
        cls.vehicle_dict[id]['route'] = None
        cls.vehicle_dict[id]['type']=type

        if type == 1:
            cls.vehicle_dict[id]['length']=4.25
            cls.vehicle_dict[id]['width']=2.1
            cls.vehicle_dict[id]['max_v']=130
            cls.vehicle_dict[id]['max_acc']=4
            cls.vehicle_dict[id]['max_dec'] = 1
            # CheryTiggoSUV
        elif type == 2:
            cls.vehicle_dict[id]['length'] = 6.33
            cls.vehicle_dict[id]['width'] = 2.64
            cls.vehicle_dict[id]['max_v'] = 100
            cls.vehicle_dict[id]['max_acc'] = 2.8
            cls.vehicle_dict[id]['max_dec'] = 1
            # FiatExpressCar
        elif type == 3:
            cls.vehicle_dict[id]['length'] = 5.36
            cls.vehicle_dict[id]['width'] = 2.17
            cls.vehicle_dict[id]['max_v'] = 120
            cls.vehicle_dict[id]['max_acc'] = 3.5
            cls.vehicle_dict[id]['max_dec'] = 1
            # VolkswagenPickUpCar
        elif type == 4:
            cls.vehicle_dict[id]['length'] = 5.15
            cls.vehicle_dict[id]['width'] = 2.04
            cls.vehicle_dict[id]['max_v'] = 115
            cls.vehicle_dict[id]['max_acc'] = 3.4
            cls.vehicle_dict[id]['max_dec'] = 1
            # DodgePoliceCar
        elif type == 5:
            cls.vehicle_dict[id]['length'] = 4.55
            cls.vehicle_dict[id]['width'] = 2.07
            cls.vehicle_dict[id]['max_v'] = 110
            cls.vehicle_dict[id]['max_acc'] = 3.1
            cls.vehicle_dict[id]['max_dec'] = 1
            # InfinitEssence
        elif type == 6:
            cls.vehicle_dict[id]['length'] = 4.82
            cls.vehicle_dict[id]['width'] = 1.05
            cls.vehicle_dict[id]['max_v'] = 105
            cls.vehicle_dict[id]['max_acc'] = 3.2
            cls.vehicle_dict[id]['max_dec'] = 1
            # LynkCoCar
        elif type == 7:
            cls.vehicle_dict[id]['length'] = 12.2
            cls.vehicle_dict[id]['width'] = 2.85
            cls.vehicle_dict[id]['max_v'] = 100
            cls.vehicle_dict[id]['max_acc'] = 2
            cls.vehicle_dict[id]['max_dec'] = 1
            # MercedesBenzCitarBus
        elif type == 8:
            cls.vehicle_dict[id]['length'] = 9.7
            cls.vehicle_dict[id]['width'] = 2.88
            cls.vehicle_dict[id]['max_v'] = 80
            cls.vehicle_dict[id]['max_acc'] = 2.3
            cls.vehicle_dict[id]['max_dec'] = 1
            # FordB760SchoolBus
        elif type == 9:
            cls.vehicle_dict[id]['length'] = 3.2
            cls.vehicle_dict[id]['width'] = 1.56
            cls.vehicle_dict[id]['max_v'] = 90
            cls.vehicle_dict[id]['max_acc'] = 2.5
            cls.vehicle_dict[id]['max_dec'] = 1
            # DaihatsuHijetVan
        elif type == 10:
            cls.vehicle_dict[id]['length'] = 9.22
            cls.vehicle_dict[id]['width'] = 3.06
            cls.vehicle_dict[id]['max_v'] = 95
            cls.vehicle_dict[id]['max_acc'] = 2.7
            cls.vehicle_dict[id]['max_dec'] = 1
            # ScaniaConcreteTruck
        elif type == 11:
            cls.vehicle_dict[id]['length'] = 8.2
            cls.vehicle_dict[id]['width'] = 2.94
            cls.vehicle_dict[id]['max_v'] = 100
            cls.vehicle_dict[id]['max_acc'] = 2.9
            cls.vehicle_dict[id]['max_dec'] = 1
            # ScanlaE402FireTruck
        elif type == 12:
            cls.vehicle_dict[id]['length'] = 8.97
            cls.vehicle_dict[id]['width'] = 3.47
            cls.vehicle_dict[id]['max_v'] = 98
            cls.vehicle_dict[id]['max_acc'] = 2.95
            cls.vehicle_dict[id]['max_dec'] = 1
            # MANTruck

        return id
