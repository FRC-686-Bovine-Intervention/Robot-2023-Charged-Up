package frc.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;

public class OdometryCommand {
    public OdometryCommand() {}

    private Pose2d          resetPose;
    public Pose2d           getResetPose()                  {return resetPose;}
    public OdometryCommand  setResetPose(Pose2d resetPose)  {this.resetPose = resetPose; return this;}
}
