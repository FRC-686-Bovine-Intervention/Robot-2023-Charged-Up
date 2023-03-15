package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotConfiguration;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmStatus.ArmState;

public class BlankAutoMode extends AutoMode {
    public BlankAutoMode() {
        startConfiguration = new RobotConfiguration(new Pose2d(), ArmPose.Preset.AUTO_START, ArmState.Hold);
    }
}
