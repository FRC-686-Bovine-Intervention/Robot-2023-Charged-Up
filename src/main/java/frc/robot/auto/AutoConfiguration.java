package frc.robot.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmStatus.ArmState;

public class AutoConfiguration {
    public enum StartPosition {
        Wall,
        Center,
        Loading
    }
    public enum GamePiece {
        Cone,
        Cube
    }
    public final StartPosition startingPosition;
    public final GamePiece startingPiece;
    public final GamePiece
    public final Pose2d initialPose;
    public final ArmPose.Preset initialArmPose;
    public final ArmState initalArmState;

    public AutoConfiguration() {
        this(new Pose2d(), ArmPose.Preset.AUTO_START, ArmState.Hold);
    }
    public AutoConfiguration(Pose2d initialPose, ArmPose.Preset initialArmPose, ArmState initalArmState) {
        this.initialPose = initialPose;
        this.initialArmPose = initialArmPose;
        this.initalArmState = initalArmState;
    }

    public AutoConfiguration log(Logger logger, String prefix) {
        logger.recordOutput(prefix + "/Initial Robot Pose", initialPose);
        logger.recordOutput(prefix + "/Initial Arm Pose", initialArmPose != null ? initialArmPose.name() : "null");
        logger.recordOutput(prefix + "/Initial Arm State", initalArmState != null ? initalArmState.name() : "null");
        return this;
    }
}
