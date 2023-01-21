package frc.robot.subsystems.driverAssist;

import edu.wpi.first.math.geometry.Pose2d;

public class DriverAssistCommand {
    public DriverAssistCommand() {this(new Pose2d());}
    public DriverAssistCommand(Pose2d targetPose) {
        setTargetPose(targetPose);
    }

    private Pose2d targetPose = new Pose2d();
    public Pose2d getTargetPose() {return targetPose;}
    public DriverAssistCommand setTargetPose(Pose2d targetPose) {this.targetPose = targetPose; return this;}
}
