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
from ArmKinematics import ArmKinematics
from unyt import meter, inch
import write_arm_config
import write_arm_presets

if __name__ == "__main__":
    write_arm_config.main()
    write_arm_presets.main()


    arm_config = json.loads(open("src/main/deploy/arm_config.json", "r").read())
    solver_config = json.loads(open("src/main/deploy/solver_config.json", "r").read())
    constraints = json.loads(open("src/main/deploy/constraints.json", "r").read())

    preset_pose_inches = json.loads(open("src/main/deploy/arm_preset_poses.json", "r").read())
    # convert all Imperial inputs to Metric
    preset_poses = preset_pose_inches.copy()
    for key in preset_poses:
        preset_poses[key]["xy"] = (preset_pose_inches[key]["xy"]*inch).to("meter").value

    kinematics = ArmKinematics(arm_config["origin"][0], arm_config["origin"][1], arm_config["shoulder"]["length"],
                               arm_config["elbow"]["length"] + arm_config["wrist"]["length"])

    solver = Solver(solver_config, arm_config, constraints)

    q = {}
    passed = {}
    for pose in preset_poses:
        q[pose] = False
    for pose in preset_poses:
        passed[pose] = q

    for start in preset_poses:
        for final in preset_poses:

            [start_theta1, start_theta2, valid] = kinematics.inverse_kinematics([preset_poses[start]["xy"][0]], [preset_poses[start]["xy"][1]])
            [final_theta1, final_theta2, valid] = kinematics.inverse_kinematics([preset_poses[final]["xy"][0]], [preset_poses[final]["xy"][1]])

            if (start == final):
                result = (0.0, start_theta1, start_theta2)

            else:
                request = {
                    "initial": [start_theta1[0], start_theta2[0]],
                    "final": [final_theta1[0], final_theta2[0]],
                    "constraintOverrides": [],
                }
                result = solver.solve(request)

            passed[start][final] = (result is not None)

            if result is not None:

                json_output = {"startPos": start, "finalPos": final, "totalTime": result[0], "grannyFactor": 1.25,
                               "theta1": result[1], "theta2": result[2]}

                filename = "src/main/deploy/paths/arm_path_{0}_{1}.json".format(
                    preset_poses[start]["fileIdx"], preset_poses[final]["fileIdx"])
                with open(filename, "w") as outfile:
                    json.dump(json_output, outfile, indent=2)

                # print("DT =", result[0] / (len(result[1]) - 1))
                # plot(result, arm_config)

        # break

    for start in preset_poses:
        for final in preset_poses:
            if not passed[start][final]:
                print("Failed to create path for " + start + ", " + final)
