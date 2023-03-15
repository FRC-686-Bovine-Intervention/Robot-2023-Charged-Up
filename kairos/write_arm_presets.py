import json

from ArmPresets import ArmPresetPoses
from ArmKinematics import ArmKinematics
from unyt import meter, inch

def main():
    arm_config = json.loads(open("src/main/deploy/arm_config.json", "r").read())

    kinematics = ArmKinematics(arm_config["origin"][0], arm_config["origin"][1], arm_config["shoulder"]["length"],
                               arm_config["elbow"]["length"] + arm_config["wrist"]["length"])

    preset_poses = ArmPresetPoses.get_preset_poses(arm_config["bumper_width_inches"])

    # convert all Imperial inputs to Metric
    for key in preset_poses:

        theta1 = preset_poses[key]["theta1"]
        theta2 = preset_poses[key]["theta2"]
        theta = [[theta1, theta2]]
        [elbow, endEffector] = kinematics.forward_kinematics(theta)

        preset_poses[key]["xy"] = ([endEffector[0][0], endEffector[1][0]] * meter).to("inch").value.tolist()

    with open("src/main/deploy/arm_preset_poses.json", "w") as outfile:
        json.dump(preset_poses, outfile, indent=2)

if __name__ == "__main__":
    main()