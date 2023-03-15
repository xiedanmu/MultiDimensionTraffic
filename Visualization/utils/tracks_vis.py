#!/usr/bin/env python
import random

import matplotlib
import matplotlib.patches
import matplotlib.transforms
import numpy as np

from .dataset_types import Track, MotionState


def rotate_around_center(pts, center, yaw):
    return np.dot(pts - center, np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])) + center


def polygon_xy_from_motionstate(ms, width, length):
    assert isinstance(ms, MotionState)
    lowleft = (ms.x - length / 2., ms.y - width / 2.)
    lowright = (ms.x + length / 2., ms.y - width / 2.)
    upright = (ms.x + length / 2., ms.y + width / 2.)
    upleft = (ms.x - length / 2., ms.y + width / 2.)
    return rotate_around_center(np.array([lowleft, lowright, upright, upleft]), np.array([ms.x, ms.y]), yaw=ms.psi_rad)

def polygon_xy_from_motionstate_new(ms, width, length):
    assert isinstance(ms, MotionState)
    lowleft = (ms.x - length, ms.y - width / 2.)
    lowright = (ms.x, ms.y - width / 2.)
    upright = (ms.x, ms.y + width / 2.)
    upleft = (ms.x - length, ms.y + width / 2.)
    return rotate_around_center(np.array([lowleft, lowright, upright, upleft]), np.array([ms.x, ms.y]), yaw=ms.psi_rad)


def polygon_xy_from_motionstate_pedest(ms, width, length):
    assert isinstance(ms, MotionState)
    lowleft = (ms.x - length / 2., ms.y - width / 2.)
    lowright = (ms.x + length / 2., ms.y - width / 2.)
    upright = (ms.x + length / 2., ms.y + width / 2.)
    upleft = (ms.x - length / 2., ms.y + width / 2.)
    return np.array([lowleft, lowright, upright, upleft])


def update_objects_plot(timestamp, patches_dict, text_dict, axes, track_dict=None, pedest_dict=None):
    if track_dict is not None:

        for key, value in track_dict.items():
            assert isinstance(value, Track)
            if value.time_stamp_ms_first <= timestamp <= value.time_stamp_ms_last:
                # object is visible

                try:
                    ms = value.motion_states[timestamp]
                except:
                    continue

                assert isinstance(ms, MotionState)

                if key not in patches_dict:
                    width = value.width
                    length = value.length

                    rect = matplotlib.patches.Polygon(polygon_xy_from_motionstate(ms, width, length), closed=True,
                                                      zorder=20)
                    patches_dict[key] = rect
                    axes.add_patch(rect)
                    text_dict[key] = axes.text(ms.x, ms.y + 2, str(key), horizontalalignment='center', zorder=30)
                else:
                    width = value.width
                    length = value.length
                    patches_dict[key].set_xy(polygon_xy_from_motionstate(ms, width, length))
                    text_dict[key].set_position((ms.x, ms.y + 2))
            else:
                if key in patches_dict:
                    patches_dict[key].remove()
                    patches_dict.pop(key)
                    text_dict[key].remove()
                    text_dict.pop(key)

    if pedest_dict is not None:

        for key, value in pedest_dict.items():
            assert isinstance(value, Track)
            if value.time_stamp_ms_first <= timestamp <= value.time_stamp_ms_last:
                # object is visible
                ms = value.motion_states[timestamp]
                assert isinstance(ms, MotionState)

                if key not in patches_dict:
                    width = 1.5
                    length = 1.5

                    rect = matplotlib.patches.Polygon(polygon_xy_from_motionstate_pedest(ms, width, length),
                                                      closed=True, zorder=20, color='red')
                    patches_dict[key] = rect
                    axes.add_patch(rect)
                    text_dict[key] = axes.text(ms.x, ms.y + 2, str(key), horizontalalignment='center', zorder=30)
                else:
                    width = 1.5
                    length = 1.5
                    patches_dict[key].set_xy(polygon_xy_from_motionstate_pedest(ms, width, length))
                    text_dict[key].set_position((ms.x, ms.y + 2))
            else:
                if key in patches_dict:
                    patches_dict[key].remove()
                    patches_dict.pop(key)
                    text_dict[key].remove()
                    text_dict.pop(key)

def update_objects_plot_per_stamp(timestamp, patches_dict, text_dict, foe_dict,axes, track_dict=None, pedest_dict=None):
    if track_dict is not None:

        for key, value in track_dict.items():
            assert isinstance(value, Track)

            # object is visible

            try:
                ms = value.motion_states[timestamp]
            except:
                continue

            assert isinstance(ms, MotionState)

            if key not in patches_dict:
                width = value.width
                length = value.length
                r = random.random()
                b = random.random()
                g = random.random()
                color = (r, g, b)
                rect = matplotlib.patches.Polygon(polygon_xy_from_motionstate_new(ms, width, length), closed=True,
                                                  zorder=20,color=color)
                patches_dict[key] = rect
                axes.add_patch(rect)
                text_dict[key] = axes.text(ms.x, ms.y, str(key), horizontalalignment='center', zorder=30)

            else:
                width = value.width
                length = value.length
                foe_id = value.foe_vehicle_id
                patches_dict[key].set_xy(polygon_xy_from_motionstate_new(ms, width, length))
                text_dict[key].set_position((ms.x, ms.y))

                if foe_id !='':
                    if key in list(foe_dict.keys()):
                        foe_dict[key].remove()
                        foe_dict.pop(key)
                        foe_dict[key] = axes.text(ms.x+4, ms.y, str(foe_id), horizontalalignment='center',
                                                  zorder=30,color='r')
                        # foe_dict[key].set_position((ms.x + 4, ms.y + 2))
                    else:
                        foe_dict[key] = axes.text(ms.x+4, ms.y, str(foe_id), horizontalalignment='center',
                                                  zorder=30,color='r')
                else:
                    if key in list(foe_dict.keys()):
                        foe_dict[key].remove()
                        foe_dict.pop(key)


        for key in list(patches_dict.keys()):
            if key not in track_dict.keys():
                # del patches_dict[key]
                # del text_dict[key]
                patches_dict[key].remove()
                patches_dict.pop(key)
                text_dict[key].remove()
                text_dict.pop(key)


    if pedest_dict is not None:

        for key, value in pedest_dict.items():
            assert isinstance(value, Track)
            if value.time_stamp_ms_first <= timestamp <= value.time_stamp_ms_last:
                # object is visible
                ms = value.motion_states[timestamp]
                assert isinstance(ms, MotionState)

                if key not in patches_dict:
                    width = 1.5
                    length = 1.5

                    rect = matplotlib.patches.Polygon(polygon_xy_from_motionstate_pedest(ms, width, length),
                                                      closed=True, zorder=20, color='red')
                    patches_dict[key] = rect
                    axes.add_patch(rect)
                    text_dict[key] = axes.text(ms.x, ms.y + 2, str(key), horizontalalignment='center', zorder=30)
                else:
                    width = 1.5
                    length = 1.5
                    patches_dict[key].set_xy(polygon_xy_from_motionstate_pedest(ms, width, length))
                    text_dict[key].set_position((ms.x, ms.y + 2))
            else:
                if key in patches_dict:
                    patches_dict[key].remove()
                    patches_dict.pop(key)
                    text_dict[key].remove()
                    text_dict.pop(key)
