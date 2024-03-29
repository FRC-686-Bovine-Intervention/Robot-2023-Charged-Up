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
    HOLD = 9


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
            "auto_start": {"fileIdx": ArmPresetEnum.AUTO_START, "theta1": -2.64, "theta2": -0.940},
            "defense": {"fileIdx": ArmPresetEnum.DEFENSE, "theta1": -2.933, "theta2": -0.815},
            "intake": {"fileIdx": ArmPresetEnum.INTAKE, "theta1": -2.30, "theta2": -0.99},
            "hold": {"fileIdx": ArmPresetEnum.HOLD, "theta1": -1.591, "theta2": 1.271},
            "double_substation": {"fileIdx": ArmPresetEnum.DOUBLE_SUBSTATION, "theta1": -0.898, "theta2": 0.29},
            "score_hybrid": {"fileIdx": ArmPresetEnum.SCORE_HYBRID, "theta1": -2.688, "theta2": -0.516},
            "score_mid_cube": {"fileIdx": ArmPresetEnum.SCORE_MID_CUBE, "theta1": -1.800, "theta2": 0.284},
            "score_high_cube": {"fileIdx": ArmPresetEnum.SCORE_HIGH_CUBE, "theta1": -0.727, "theta2": 0.322},
            "score_mid_cone": {"fileIdx": ArmPresetEnum.SCORE_MID_CONE, "theta1": -1.763, "theta2": 0.40},
            "score_high_cone": {"fileIdx": ArmPresetEnum.SCORE_HIGH_CONE, "theta1": -0.733, "theta2": 0.55}
        }

        return preset
