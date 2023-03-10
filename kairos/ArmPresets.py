import numpy as np
from enum import IntEnum


class ArmPresetEnum(IntEnum):
    DEFENSE = 0
    INTAKE = 1
    DOUBLE_SUBSTATION = 2
    SCORE_HYBRID = 3
    SCORE_MID_CUBE = 4
    SCORE_HIGH_CUBE = 5
    SCORE_MID_CONE = 6
    SCORE_HIGH_CONE = 7


class ArmPresetPoses:
    @staticmethod
    def get_preset_poses(bumper_width_inches):
        # all values in inches
        # TODO: convert to json

        scoring_margin = 6
        shelf_depth = 17.0
        dbl_substation_shelf_depth = 14.0

        # preset = [None] * ((ArmPresetEnum.SCORE_HIGH_CONE).value+1)
        preset = {
            "defense":           {"fileIdx": ArmPresetEnum.DEFENSE, "xy": np.array([ 6.0, 25.0]).tolist()},
            "intake":            {"fileIdx": ArmPresetEnum.INTAKE, "xy": np.array([14, 15.0]).tolist()},
            "double_substation": {"fileIdx": ArmPresetEnum.DOUBLE_SUBSTATION, "xy": (np.array([bumper_width_inches/2 + dbl_substation_shelf_depth/2, 37.3])).tolist()},
            "score_hybrid":      {"fileIdx": ArmPresetEnum.SCORE_HYBRID, "xy": (np.array([bumper_width_inches/2 + shelf_depth/2, 0]) + np.array([0, scoring_margin])).tolist()},
            "score_mid_cube":    {"fileIdx": ArmPresetEnum.SCORE_MID_CUBE, "xy": (np.array([bumper_width_inches/2 + 14.25  + shelf_depth/2, 23.5]) + np.array([0, scoring_margin])).tolist()},
            "score_high_cube":   {"fileIdx": ArmPresetEnum.SCORE_HIGH_CUBE, "xy": (np.array([bumper_width_inches/2 + 31.625 + shelf_depth/2, 35.5]) + np.array([0, scoring_margin])).tolist()},
            "score_mid_cone":    {"fileIdx": ArmPresetEnum.SCORE_MID_CONE, "xy": (np.array([bumper_width_inches/2 + 22.75, 34.0]) + np.array([0, scoring_margin])).tolist()},
            "score_high_cone":   {"fileIdx": ArmPresetEnum.SCORE_HIGH_CONE, "xy": (np.array([bumper_width_inches/2 + 39.75, 46.0]) + np.array([0, scoring_margin])).tolist()}
        }

        return preset
