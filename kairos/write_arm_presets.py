import json

from ArmPresets import ArmPresetPoses
from ArmKinematics import ArmKinematics
from unyt import meter, inch

if __name__ == "__main__":
    arm_config = json.loads(open("src/main/deploy/arm_config.json", "r").read())

    kinematics = ArmKinematics(arm_config["origin"][0], arm_config["origin"][1], arm_config["shoulder"]["length"],
                               arm_config["elbow"]["length"] + arm_config["wrist"]["length"])

    bumper_width_inches = arm_config["bumper_width_inches"]
    preset_pose_inches = ArmPresetPoses.get_preset_poses(bumper_width_inches)

    # convert all Imperial inputs to Metric
    preset_poses = preset_pose_inches.copy()
    for key in preset_poses:
        xy_inch = preset_poses[key]["xy"]
        xy_m = (xy_inch * inch).to("meter").value

        [theta1, theta2, valid] = kinematics.inverse_kinematics([xy_m[0]], [xy_m[1]])

        preset_pose_inches[key]["theta1"] = theta1[0]
        preset_pose_inches[key]["theta2"] = theta2[0]

    with open("src/main/deploy/arm_preset_poses.json", "w") as outfile:
        json.dump(preset_pose_inches, outfile, indent=2)
