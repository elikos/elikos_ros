import pyopengv as _pyopengv
import numpy as np


def epnp_multi_camera(bearings, coordinates, camera_translations, camera_rotations):
    # type: (list[np.ndarray], list[np.ndarray], np.ndarray, np.ndarray)->np.ndarray
    return _pyopengv.epnp_multi_camera(bearings, coordinates, camera_translations, camera_rotations, "upnp")