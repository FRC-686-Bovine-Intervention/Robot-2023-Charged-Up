package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmStatus.ArmState;

public class RobotConfiguration {
    public final Pose2d robotPose;
    public final ArmPose.Preset armPose;
    public final ArmState armState;
    
    public RobotConfiguration() {this(new Pose2d(), null, ArmState.DEFAULT);}
    public RobotConfiguration(Pose2d robotPose, ArmPose.Preset armPose, ArmState armState) {
        this.robotPose = robotPose;
        this.armPose = armPose;
        this.armState = armState;
    }
}
