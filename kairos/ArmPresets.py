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
    AUTO_START = 8


class ArmPresetPoses:
    @staticmethod
    def get_preset_poses(bumper_width_inches):
        # all values in inches
        # TODO: convert to json

        scoring_margin = 12.0
        shelf_depth = 17.0
        dbl_substation_shelf_depth = 14.0

        # preset = [None] * ((ArmPresetEnum.SCORE_HIGH_CONE).value+1)
        preset = {
            # "auto_start":        {"fileIdx": ArmPresetEnum.AUTO_START, "xy": np.array([13.483, 37.597]).tolist()},
            # "defense":           {"fileIdx": ArmPresetEnum.DEFENSE, "xy": np.array([13.916, 32.975]).tolist()},
            # "intake":            {"fileIdx": ArmPresetEnum.INTAKE, "xy": np.array([13.790, 18.725]).tolist()},
            # "double_substation": {"fileIdx": ArmPresetEnum.DOUBLE_SUBSTATION, "xy": [54.0, 44.0]},
            # # "score_hybrid":      {"fileIdx": ArmPresetEnum.SCORE_HYBRID, "xy": (np.array([bumper_width_inches/2 + shelf_depth/2, 0]) + np.array([0, 6+scoring_margin])).tolist()},
            # # "score_mid_cube":    {"fileIdx": ArmPresetEnum.SCORE_MID_CUBE, "xy": (np.array([bumper_width_inches/2 + 14.25  + shelf_depth/2, 23.5]) + np.array([0, scoring_margin])).tolist()},
            # # "score_high_cube":   {"fileIdx": ArmPresetEnum.SCORE_HIGH_CUBE, "xy": (np.array([bumper_width_inches/2 + 31.625 + shelf_depth/2, 35.5]) + np.array([0, scoring_margin])).tolist()},
            # # "score_mid_cone":    {"fileIdx": ArmPresetEnum.SCORE_MID_CONE, "xy": (np.array([bumper_width_inches/2 + 22.75, 34.0]) + np.array([0, scoring_margin])).tolist()},
            # # "score_high_cone":   {"fileIdx": ArmPresetEnum.SCORE_HIGH_CONE, "xy": (np.array([bumper_width_inches/2 + 39.75, 46.0]) + np.array([0, scoring_margin])).tolist()}
            # "score_hybrid": {"fileIdx": ArmPresetEnum.SCORE_HYBRID, "xy": (np.array([16, 33])).tolist()},
            # "score_mid_cube": {"fileIdx": ArmPresetEnum.SCORE_MID_CONE, "xy": (np.array([37, 31])).tolist()},
            # "score_high_cube": {"fileIdx": ArmPresetEnum.SCORE_HIGH_CONE, "xy": (np.array([57, 42])).tolist()},
            # "score_mid_cone": {"fileIdx": ArmPresetEnum.SCORE_MID_CUBE, "xy": (np.array([34, 37])).tolist()},
            # "score_high_cone": {"fileIdx": ArmPresetEnum.SCORE_HIGH_CUBE, "xy": (np.array([52, 48.5])).tolist()}

            "auto_start": {"fileIdx": ArmPresetEnum.AUTO_START, "theta1": -2.340, "theta2": -0.422},
            "defense": {"fileIdx": ArmPresetEnum.DEFENSE, "theta1": -2.26, "theta2": 0.160},
            "intake": {"fileIdx": ArmPresetEnum.INTAKE, "theta1": -2.048, "theta2": -0.524},
            "double_substation": {"fileIdx": ArmPresetEnum.DOUBLE_SUBSTATION, "theta1": -0.603, "theta2": 0.472},
            "score_hybrid": {"fileIdx": ArmPresetEnum.SCORE_HYBRID, "theta1": -2.100, "theta2": 0.000},
            "score_mid_cube": {"fileIdx": ArmPresetEnum.SCORE_MID_CUBE, "theta1": -1.411, "theta2": 0.485},
            "score_high_cube": {"fileIdx": ArmPresetEnum.SCORE_HIGH_CUBE, "theta1": -0.679, "theta2": 0.442},
            "score_mid_cone": {"fileIdx": ArmPresetEnum.SCORE_MID_CONE, "theta1": -1.300, "theta2": 0.938},
            "score_high_cone": {"fileIdx": ArmPresetEnum.SCORE_HIGH_CONE, "theta1": -0.487, "theta2": 0.852}
        }

        return preset
