import numpy as np
from enum import IntEnum


class Location(IntEnum):
    DEFENSE = 1
    INTAKE = 2
    DBL_SUBSTATION_INTAKE = 3
    LOW_SHELF_SCORING = 4
    MID_SHELF_SCORING = 5
    HIGH_SHELF_SCORING = 6
    MID_POLE_SCORING = 7
    HIGH_POLE_SCORING = 8


class PresetLocations:
    @staticmethod
    def get_locations(center_to_side_bumper):
        # all values in inches
        # TODO: convert to json

        scoring_margin = 6
        shelf_depth = 17.0
        dbl_substation_shelf_depth = 14.0

        preset = [None] * ((Location.HIGH_POLE_SCORING).value+1)
        preset[Location.DEFENSE]                = np.array([ 0.0, 25.0])
        preset[Location.INTAKE]                 = np.array([16.3, 11.0])
        preset[Location.DBL_SUBSTATION_INTAKE]  = np.array([center_to_side_bumper + dbl_substation_shelf_depth/2, 37.3])
        preset[Location.LOW_SHELF_SCORING]      = np.array([center_to_side_bumper + shelf_depth/2, 0]) + np.array([0, scoring_margin])
        preset[Location.MID_SHELF_SCORING]      = np.array([center_to_side_bumper + 14.25  + shelf_depth/2, 23.5]) + np.array([0, scoring_margin])
        preset[Location.HIGH_SHELF_SCORING]     = np.array([center_to_side_bumper + 31.625 + shelf_depth/2, 35.5]) + np.array([0, scoring_margin])
        preset[Location.MID_POLE_SCORING]       = np.array([center_to_side_bumper + 22.75, 34.0]) + np.array([0, scoring_margin])
        preset[Location.HIGH_POLE_SCORING]      = np.array([center_to_side_bumper + 39.75, 46.0]) + np.array([0, scoring_margin])

        return preset
