import json

from ArmPresets import ArmPresetPoses

if __name__ == "__main__":
    arm_config = json.loads(open("src/main/deploy/arm_config.json", "r").read())

    center_to_side_bumper_inches = arm_config["center_to_side_bumper_inches"]
    preset_pose_inches = ArmPresetPoses.get_preset_poses(center_to_side_bumper_inches)

    with open("src/main/deploy/arm_preset_poses.json", "w") as outfile:
        json.dump(preset_pose_inches, outfile, indent=2)
