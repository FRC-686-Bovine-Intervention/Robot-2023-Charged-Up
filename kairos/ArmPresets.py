import numpy as np
from enum import IntEnum


class ArmPresetEnum(IntEnum):
    DEFENSE = 1
    INTAKE = 2
    DOUBLE_SUBSTATION = 3
    SCORE_HYBRID = 4
    SCORE_MID_CUBE = 5
    SCORE_HIGH_CUBE = 6
    SCORE_MID_CONE = 7
    SCORE_HIGH_CONE = 8


class ArmPresetPoses:
    @staticmethod
    def get_preset_poses(center_to_side_bumper):
        # all values in inches
        # TODO: convert to json

        scoring_margin = 6
        shelf_depth = 17.0
        dbl_substation_shelf_depth = 14.0

        # preset = [None] * ((ArmPresetEnum.SCORE_HIGH_CONE).value+1)
        preset = {
            "defense":           {"fileIdx": ArmPresetEnum.DEFENSE, "xy": np.array([ 0.0, 25.0]).tolist()},
            "intake":            {"fileIdx": ArmPresetEnum.INTAKE, "xy": np.array([16.3, 11.0]).tolist()},
            "double_substation": {"fileIdx": ArmPresetEnum.DOUBLE_SUBSTATION, "xy": (np.array([center_to_side_bumper + dbl_substation_shelf_depth/2, 37.3])).tolist()},
            "score_hybrid":      {"fileIdx": ArmPresetEnum.SCORE_HYBRID, "xy": (np.array([center_to_side_bumper + shelf_depth/2, 0]) + np.array([0, scoring_margin])).tolist()},
            "score_mid_cube":    {"fileIdx": ArmPresetEnum.SCORE_MID_CUBE, "xy": (np.array([center_to_side_bumper + 14.25  + shelf_depth/2, 23.5]) + np.array([0, scoring_margin])).tolist()},
            "score_high_cube":   {"fileIdx": ArmPresetEnum.SCORE_HIGH_CUBE, "xy": (np.array([center_to_side_bumper + 31.625 + shelf_depth/2, 35.5]) + np.array([0, scoring_margin])).tolist()},
            "score_mid_cone":    {"fileIdx": ArmPresetEnum.SCORE_MID_CONE, "xy": (np.array([center_to_side_bumper + 22.75, 34.0]) + np.array([0, scoring_margin])).tolist()},
            "score_high_cone":   {"fileIdx": ArmPresetEnum.SCORE_HIGH_CONE, "xy": (np.array([center_to_side_bumper + 39.75, 46.0]) + np.array([0, scoring_margin])).tolist()}
        }

        return preset
