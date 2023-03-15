#!/usr/bin/env python
from Abnormal_Traffic_HWP_Visualization import ModelOutput, ModelStart
from Library.package_platformAPI.ExternalInterfaceImpl import ExternalInterfaceImpl

try:
    import lanelet2

    use_lanelet2_lib = True
except ImportError:
    import warnings

    string = "Could not import lanelet2. It must be built and sourced, " + \
             "see https://github.com/fzi-forschungszentrum-informatik/Lanelet2 for details."
    warnings.warn(string)
    print("Using visualization without lanelet2.")
    use_lanelet2_lib = False
    from utils import map_vis_without_lanelet
    from utils import map_vis_in_SUMO

import argparse
import os
import time
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from utils import dataset_reader
from utils import dataset_types
from utils import map_vis_lanelet2
from utils import tracks_vis
from utils import dict_utils


def update_plot():
    #TODO
    global userdata
    result_dict=ModelOutput(userdata)

    """
    在这个位置计算新仿真步的各项数据，并以timestamp为Key存入track_dictionary（对应于move_to函数）

    ModelOutput(track_dictionary)
    涉及到的主要接口函数，比如获取前车、获取左右后方/左右前方的车，获取左、右边道路，获取某条lane上的所有车，获取车的速度加速度、
    获取当前位置到路的尽头的距离、是否是思路、获取前方红绿灯状态、获取前方所有转弯的方向、获取nextLanes、检查是不是内部道路、myAddVehicle、
    myTranslateDirectionToRoute、myChangeRoute等

    路网文件解析（除开道路形状数据以外，其它的对应与包含关系等）
    """

    global fig, timestamp, title_text, track_dictionary, patches_dict, text_dict,foe_dict, axes, pedestrian_dictionary
    # update text and tracks based on current timestamp
    assert (timestamp <= timestamp_max), "timestamp=%i" % timestamp
    assert (timestamp >= timestamp_min), "timestamp=%i" % timestamp
    #assert (timestamp % dataset_types.DELTA_TIMESTAMP_MS == 0), "timestamp=%i" % timestamp

    track_dictionary=dataset_reader.read_tracks_per_stamp(result_dict,userdata["time"])

    percentage = (float(timestamp) / timestamp_max) * 100
    title_text.set_text("\nts = {} / {} ({:.2f}%)".format(timestamp, timestamp_max, percentage))
    tracks_vis.update_objects_plot_per_stamp(timestamp, patches_dict, text_dict,foe_dict, axes,
                                   track_dict=track_dictionary, pedest_dict=pedestrian_dictionary)
    fig.canvas.draw()


def start_playback():
    global userdata
    global timestamp, timestamp_min, timestamp_max, playback_stopped
    playback_stopped = False
    plt.ion()
    while timestamp < timestamp_max and not playback_stopped:
        timestamp += dataset_types.DELTA_TIMESTAMP_MS
        userdata["time"] = timestamp
        start_time = time.time()
        update_plot()
        end_time = time.time()
        diff_time = end_time - start_time
        plt.pause(max(0.001, dataset_types.DELTA_TIMESTAMP_MS / 1000. - diff_time))
        #plt.pause(0.001)
    plt.ioff()


class FrameControlButton(object):
    def __init__(self, position, label):
        self.ax = plt.axes(position)
        self.label = label
        self.button = Button(self.ax, label)
        self.button.on_clicked(self.on_click)

    def on_click(self, event):
        global timestamp, timestamp_min, timestamp_max, playback_stopped

        if self.label == "play":
            if not playback_stopped:
                return
            else:
                start_playback()
                return
        playback_stopped = True
        if self.label == "<<":
            timestamp -= 10 * dataset_types.DELTA_TIMESTAMP_MS
        elif self.label == "<":
            timestamp -= dataset_types.DELTA_TIMESTAMP_MS
        elif self.label == ">":
            timestamp += dataset_types.DELTA_TIMESTAMP_MS
        elif self.label == ">>":
            timestamp += 10 * dataset_types.DELTA_TIMESTAMP_MS
        timestamp = min(timestamp, timestamp_max)
        timestamp = max(timestamp, timestamp_min)
        update_plot()


if __name__ == "__main__":

    # provide data to be visualized
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario_name", type=str, help="Name of the scenario (to identify map and folder for track "
                                                        "files)",default='Cross-Dislocation', nargs="?")
    parser.add_argument("track_file_number", type=int, help="Number of the track file (int)", default=0, nargs="?")
    parser.add_argument("load_mode", type=str, help="Dataset to load (vehicle, pedestrian, or both)", default="both",
                        nargs="?")
    parser.add_argument("--start_timestamp", type=int, nargs="?")
    parser.add_argument("--lat_origin", type=float,
                        help="Latitude of the reference point for the projection of the lanelet map (float)",
                        default=0.0, nargs="?")
    parser.add_argument("--lon_origin", type=float,
                        help="Longitude of the reference point for the projection of the lanelet map (float)",
                        default=0.0, nargs="?")
    args = parser.parse_args()

    if args.scenario_name is None:
        raise IOError("You must specify a scenario. Type --help for help.")
    if args.load_mode != "vehicle" and args.load_mode != "pedestrian" and args.load_mode != "both":
        raise IOError("Invalid load command. Use 'vehicle', 'pedestrian', or 'both'")

    # check folders and files
    error_string = ""

    # root directory is where main_visualize_data.py file located
    # i.e. the root directory of this project
    root_dir = os.path.dirname(os.path.abspath(__file__))

    tracks_dir = os.path.join(root_dir, "recorded_trackfiles")
    maps_dir = os.path.join(root_dir, "maps")

    lanelet_map_ending = ".net.xml"
    lanelet_map_file = os.path.join(maps_dir, args.scenario_name + lanelet_map_ending)

    scenario_dir = os.path.join(tracks_dir, args.scenario_name)

    track_file_name = os.path.join(
        scenario_dir,
        "vehicle_tracks_" + str(args.track_file_number).zfill(3) + ".csv"
    )
    pedestrian_file_name = os.path.join(
        scenario_dir,
        "pedestrian_tracks_" + str(args.track_file_number).zfill(3) + ".csv"
    )

    if not os.path.isdir(tracks_dir):
        error_string += "Did not find track file directory \"" + tracks_dir + "\"\n"
    if not os.path.isdir(maps_dir):
        error_string += "Did not find map file directory \"" + tracks_dir + "\"\n"
    if not os.path.isdir(scenario_dir):
        error_string += "Did not find scenario directory \"" + scenario_dir + "\"\n"
    if not os.path.isfile(lanelet_map_file):
        error_string += "Did not find lanelet map file \"" + lanelet_map_file + "\"\n"
    if not os.path.isfile(track_file_name):
        error_string += "Did not find track file \"" + track_file_name + "\"\n"
    if not os.path.isfile(pedestrian_file_name):
        flag_ped = 0
    else:
        flag_ped = 1
    if error_string != "":
        error_string += "Type --help for help."
        raise IOError(error_string)

    # create a figure
    fig, axes = plt.subplots(1, 1)
    fig.canvas.set_window_title("Interaction Dataset Visualization")

    # load and draw the lanelet2 map, either with or without the lanelet2 library
    lat_origin = args.lat_origin  # origin is necessary to correctly project the lat lon values of the map to the local
    lon_origin = args.lon_origin  # coordinates in which the tracks are provided; defaulting to (0|0) for every scenario
    print("Loading map...")
    if use_lanelet2_lib:
        projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(lat_origin, lon_origin))
        laneletmap = lanelet2.io.load(lanelet_map_file, projector)
        map_vis_lanelet2.draw_lanelet_map(laneletmap, axes)
    else:
        #map_vis_without_lanelet.draw_map_without_lanelet(lanelet_map_file, axes, lat_origin, lon_origin)
        map_vis_in_SUMO.draw_map_with_SUMO(lanelet_map_file, axes)

    # load the tracks
    print("Loading tracks...")
    track_dictionary = None
    pedestrian_dictionary = None
    if args.load_mode == 'both':
        track_dictionary = dataset_reader.read_tracks(track_file_name)
        if flag_ped:
            pedestrian_dictionary = dataset_reader.read_pedestrian(pedestrian_file_name)

    elif args.load_mode == 'vehicle':
        track_dictionary = dataset_reader.read_tracks(track_file_name)
    elif args.load_mode == 'pedestrian':
        pedestrian_dictionary = dataset_reader.read_pedestrian(pedestrian_file_name)

    # timestamp_min = 1e9
    # timestamp_max = 0

    # if track_dictionary is not None:
    #     for key, track in dict_utils.get_item_iterator(track_dictionary):
    #         timestamp_min = min(timestamp_min, track.time_stamp_ms_first)
    #         timestamp_max = max(timestamp_max, track.time_stamp_ms_last)
    # else:
    #     for key, track in dict_utils.get_item_iterator(pedestrian_dictionary):
    #         timestamp_min = min(timestamp_min, track.time_stamp_ms_first)
    #         timestamp_max = max(timestamp_max, track.time_stamp_ms_last)

    timestamp_min = 0
    timestamp_max = 1e9

    if args.start_timestamp is None:
        args.start_timestamp = timestamp_min

    button_pp = FrameControlButton([0.2, 0.05, 0.05, 0.05], '<<')
    button_p = FrameControlButton([0.27, 0.05, 0.05, 0.05], '<')
    button_f = FrameControlButton([0.4, 0.05, 0.05, 0.05], '>')
    button_ff = FrameControlButton([0.47, 0.05, 0.05, 0.05], '>>')

    button_play = FrameControlButton([0.6, 0.05, 0.1, 0.05], 'play')
    button_pause = FrameControlButton([0.71, 0.05, 0.1, 0.05], 'pause')

    # storage for track visualization
    patches_dict = dict()
    text_dict = dict()
    foe_dict = dict()
    # visualize tracks
    print("Plotting...")
    timestamp = args.start_timestamp

    #程序初始化
    ExternalInterface= ExternalInterfaceImpl
    ExternalInterface.initial(lanelet_map_file)
    userdata=dict()
    userdata["VehicleDensity"]=20 #车俩密度
    userdata["time"] = timestamp
    ModelStart(userdata)
    title_text = fig.suptitle("")

    playback_stopped = True
    update_plot()
    plt.show()
