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
from PresetLocations import Location
from PresetLocations import PresetLocations
from unyt import meter, inch

if __name__ == "__main__":
    arm_config = json.loads(open("src/main/deploy/arm_config.json", "r").read())
    constraints_inches = json.loads(open("src/main/deploy/constraints.json", "r").read())
    solver_config = json.loads(open("src/main/deploy/solver_config.json", "r").read())

    center_to_side_bumper_inches = arm_config["center_to_side_bumper_inches"]
    preset_locations_inches = PresetLocations.get_locations(center_to_side_bumper_inches)

    # convert all Imperial inputs to Metric
    preset_locations = preset_locations_inches.copy()
    for loc in Location:
        preset_locations[loc] = (preset_locations_inches[loc]*inch).to("meter").value

    constraints = {}
    for key in constraints_inches:
        constraints[key] = {}
        constraints[key]["type"] = constraints_inches[key]["type"]
        constraints[key]["args"] = (constraints_inches[key]["args"].copy()*inch).to("meter").value
        # constraints[key]["args"] = [0.5, 0.0, 1.0, 0.5]
        # constraints[key]["args"] = constraints_inches[key]["args"].copy()

    solver = Solver(arm_config, constraints, solver_config)

    for start_idx in Location:
        for final_idx in Location:
            # start_idx = Location.DEFENSE
            # final_idx = Location.HIGH_POLE_SCORING

            request = {
                "start_xy": preset_locations[start_idx],
                "final_xy": preset_locations[final_idx],
                "constraintOverrides": [],
            }
            result = solver.solve(request)

            if result is not None:
                filename = "src/main/deploy/paths/arm_path_{0}_{1}.json".format(start_idx.value, final_idx.value)
                with open(filename, "w") as outfile:
                    json.dump(result, outfile)

                # print("DT =", result[0] / (len(result[1]) - 1))
                # plot(result, arm_config)

            # break
        break