# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import json
from math import pi

from Plotter import plot
from Solver import Solver
from ArmPresets import ArmPresetEnum
from ArmPresets import ArmPresetPoses
from unyt import meter, inch

if __name__ == "__main__":
    arm_config = json.loads(open("src/main/deploy/arm_config.json", "r").read())
    constraints_inches = json.loads(open("src/main/deploy/constraints.json", "r").read())
    solver_config = json.loads(open("src/main/deploy/solver_config.json", "r").read())

    bumper_width_inches = arm_config["bumper_width_inches"]
    preset_pose_inches = ArmPresetPoses.get_preset_poses(bumper_width_inches)

    # convert all Imperial inputs to Metric
    preset_poses = preset_pose_inches.copy()
    for key in preset_poses:
        preset_poses[key]["xy"] = (preset_pose_inches[key]["xy"]*inch).to("meter").value

    constraints = {}
    for key in constraints_inches:
        constraints[key] = {}
        constraints[key]["type"] = constraints_inches[key]["type"]
        constraints[key]["args"] = (constraints_inches[key]["args"]*inch).to("meter").value

    solver = Solver(arm_config, constraints, solver_config)

    for start in preset_poses:
        for final in preset_poses:

            request = {
                "start_xy": preset_poses[start]["xy"],
                "final_xy": preset_poses[final]["xy"],
                "constraintOverrides": [],
                }
            result = solver.solve(request)

            if result is not None:
                json_output = {"startPos": start, "finalPos": final, "totalTime": result[0], "theta1": result[1],
                               "theta2": result[2]}

                filename = "src/main/deploy/paths/arm_path_{0}_{1}.json".format(
                    preset_poses[start]["fileIdx"].value, preset_poses[final]["fileIdx"].value)
                with open(filename, "w") as outfile:
                    json.dump(json_output, outfile, indent=2)

                # print("DT =", result[0] / (len(result[1]) - 1))
                # plot(result, arm_config)

            # break
        # break