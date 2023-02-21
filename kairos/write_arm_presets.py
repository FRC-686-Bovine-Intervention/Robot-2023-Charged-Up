import json

from ArmPresets import ArmPresetPoses

if __name__ == "__main__":
    arm_config = json.loads(open("src/main/deploy/arm_config.json", "r").read())

    bumper_width_inches = arm_config["bumper_width_inches"]
    preset_pose_inches = ArmPresetPoses.get_preset_poses(bumper_width_inches)


    with open("src/main/deploy/arm_preset_poses.json", "w") as outfile:
        json.dump(preset_pose_inches, outfile, indent=2)
