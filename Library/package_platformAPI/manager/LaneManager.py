import xml.etree.ElementTree as xml

class LaneManager:
    #写单例模式就不要写init
    e = None
    junctionList=[]
    laneList=[]
    dead_end_lane_list=[]
    junction_element=dict()
    edge_element=dict()
    lane_element=dict()
    vehicles_on_lane=dict() #{lane1:[1,2,3],lane2:[4,5,6]}

    @classmethod
    def initial(cls,sumo_file_name):
        cls.initialJunctionList(sumo_file_name)
        cls.initialElement()
        cls.initalVehicleOnLane()

    @classmethod
    def initialJunctionList(cls,sumo_file_name):
        cls.e = xml.parse(sumo_file_name).getroot()

        #junctionList
        for node in cls.e.findall("junction"):
            cls.junction_element[node.get("id")]=node
            cls.junctionList.append(node.get("id"))

            if "dead_end"==node.get("type"):
                cls.dead_end_lane_list.extend(node.get("incLanes").split(" ", -1))

    @classmethod
    def initialElement(cls):
        for node in cls.e.findall("edge"):
            cls.edge_element[node.get("id")] = node
            for tag in node.findall("lane"):
                cls.lane_element[tag.get("id")] = tag
                cls.laneList.append(tag.get("id"))

    @classmethod
    def initalVehicleOnLane(cls):
        for lane in cls.laneList:
            cls.vehicles_on_lane[lane]=[]

    @classmethod
    def get_incoming_lanes(cls,junctionId):
        '''

        :param junctionId:string
        :return:[string,string,......,string]
        '''
        result=[]
        for node in cls.e.findall("junction"):
            if junctionId==node.get("id") and node.get("type")!='internal':
                result = node.get("incLanes").split()
                break

        return result

    @classmethod
    def get_lane_shape(cls,laneID):
        '''

        :param laneID: string
        :return: [[float,float],......,[float,float]]
        '''
        result=[]
        shape=cls.lane_element[laneID].get("shape")
        shape_str_list = shape.split(" ", -1)  # 以空格分割，分割次数：默认为 -1, 即分隔所有。
        for item in shape_str_list:
            x, y = item.split(",", -1)
            result.append((float(x),float(y)))

        return result

    @classmethod
    def get_lane_length(cls,laneID):
        return cls.lane_element[laneID].get("length")

    @classmethod
    def get_internal_lanes(cls,junctionId):
        result = []
        for node in cls.e.findall("junction"):
            if junctionId == node.get("id"):
                intLanes_str=node.get("intLanes")
                result = intLanes_str.split()
                #检查一下有没有internal junction
                for item in result:
                    try:
                        internal_junction_element=cls.junction_element[item]
                    except:
                        pass
                    else:
                        intLanes_str=internal_junction_element.get("incLanes")
                        intLanes_str_split=intLanes_str.split()
                        # 把第一个加到结果中 比如
                        # <junction id=":gneJ2_11_0" type="internal" x="0.00" y="-26.75"
                        # incLanes=":gneJ2_3_0 -gneE2_0 -gneE2_1 -gneE2_2"
                        # intLanes=":gneJ2_5_0 :gneJ2_5_1 :gneJ2_8_0 :gneJ2_8_1 :gneJ2_8_2"/>
                        result.append(intLanes_str_split[0])

                break

        return result

    @classmethod
    def get_valid_direction(cls,laneID):
        result=[]
        for node in cls.e.findall("connection"):
            from_edge=node.get("from")
            from_lane=node.get("fromLane")
            lane=from_edge+"_"+from_lane
            if lane==laneID:
                result.append(node.get("dir"))

        return result

    @classmethod
    def is_internal_lane(cls,laneID):
        #比如
        #<edge id=":gneJ2_11" function="internal">
        #   <lane id=":gneJ2_11_0" index="0" speed="13.89" length="39.64" width="3.50" shape="0.00,-26.75 0.27,-28.61 6.31,-38.69 16.39,-44.73 30.50,-46.75"/>
        #</edge>

        split_str_list = laneID.rsplit("_", 1)
        edge_id = split_str_list[0]

        edge = cls.edge_element[edge_id]
        if "internal"==edge.get("function"):
            return True
        else:
            return False

    @classmethod
    def get_left_lane(cls,laneID):
        '''

        :param laneID: string
        :return: string or None
        '''
        element = cls.lane_element[laneID]
        index=int(element.get('index'))
        split_str_list = laneID.rsplit("_", 1)
        edge_id = split_str_list[0]

        edge=cls.edge_element[edge_id]
        all_lane_element=edge.findall("lane")
        left_index=index+1
        if len(all_lane_element) - 1 >= left_index:
            for each_lane in all_lane_element:
                if int(each_lane.get('index'))==left_index:
                    return each_lane.get('id')

    @classmethod
    def get_right_lane(cls,laneID):
        '''

        :param laneID: string
        :return: string or None
        '''
        element = cls.lane_element[laneID]
        index=int(element.get('index'))
        split_str_list = laneID.rsplit("_", 1)
        edge_id = split_str_list[0]

        edge = cls.edge_element[edge_id]
        all_lane_element=edge.findall("lane")
        right_index=index-1
        if 0 <= right_index:
            for each_lane in all_lane_element:
                if int(each_lane.get('index'))==right_index:
                    return each_lane.get('id')

    @classmethod
    def is_dead_end(cls,laneID):
        """

        :param laneID: string
        :return: bool
        """
        if laneID in cls.dead_end_lane_list:
            return True
        else:
            return False


    @classmethod
    def get_lane_x_list_and_y_list_by_laneIdElemet(cls,laneID_element):
        '''

        :param laneID_element: xml中的lane id element
        :return: 该lane起始点和结束点的x的点集list和y的点集list
        '''
        x_list = list()
        y_list = list()

        shape = laneID_element.get("shape")  # 例如shape="-8.75,75.50 -10.11,65.98 -14.19,59.19 -20.98,55.11 -30.50,53.75"
        shape_str_list = shape.split(" ", -1)  # 以空格分割，分割次数：默认为 -1, 即分隔所有。
        for item in shape_str_list:
            x, y = item.split(",", -1)
            x_list.append(float(x)), y_list.append(float(y))
        return x_list, y_list

    @classmethod
    def get_lane_vehicles(cls,laneID):
        return cls.vehicles_on_lane[laneID]

    @classmethod
    def delete_vehicle_on_lane(cls,previous_lane,vehID):
        cls.vehicles_on_lane[previous_lane].remove(vehID)

    @classmethod
    def add_vehicle_on_lane(cls,lane,vehID):
        cls.vehicles_on_lane[lane].append(vehID)

    @classmethod
    def get_internal_foe_lanes(cls,laneID):

        split_str_list=laneID.split("_",-1)
        junction_id=split_str_list[0][1:]

        internal_lanes=cls.get_internal_lanes(junction_id)
        if laneID in internal_lanes:
            result=[]
            junction_e=cls.junction_element[junction_id]
            index=internal_lanes.index(laneID)

            for request in junction_e.findall("request"):
                if index!=int(request.get('index')):
                    continue
                foes=request.get("foes")[-1::-1] #反向foes序列
                foes_sequence=list(foes)
                foes_sequence=list(map(int,foes_sequence)) #都转换为int
                for index_in_foes_sequence in range(0,foes_sequence.__len__()):
                    if foes_sequence[index_in_foes_sequence]==1:
                        result.append([internal_lanes[index_in_foes_sequence],"conflict"])

                return result
        else:
            return None

    @classmethod
    def get_all_next_lanes(cls,laneID,dir):
        split_str_list = laneID.rsplit("_", 1) #rsplit("_",1) 中的1，表示从后面分割1个出来，如果是2，那么就是分割两部分出来
        my_from=split_str_list[0]
        my_fromLane=split_str_list[1]
        for connection in cls.e.findall('connection'):
            if connection.get("from")==my_from and connection.get("fromLane")==my_fromLane and connection.get("dir")==dir:
                result=[]
                via=connection.get("via")
                if via is not None:
                    result.append(via)
                    split_str_list = via.rsplit("_", 1)
                    my_via_from = split_str_list[0]
                    my_via_fromLane = split_str_list[1]

                    for item in cls.e.findall('connection'):
                        if item.get("from") == my_via_from and item.get("fromLane") == my_via_fromLane and item.get(
                                "dir") == dir and item.get("via") is not None:
                            result.append(item.get("via"))

                result.append(connection.get("to")+"_"+connection.get("toLane"))
                return [result]

        return None

