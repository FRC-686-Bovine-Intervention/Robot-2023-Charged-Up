package frc.robot.subsystems.driverAssist;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.driverAssist.DriverAssistStatus.DriverAssistState;

public class DriverAssistCommand {
    public DriverAssistCommand() {this(DriverAssistState.Disabled);}
    public DriverAssistCommand(DriverAssistState driverAssistState) {this(driverAssistState, new Pose2d());}
    public DriverAssistCommand(DriverAssistState driverAssistState, Pose2d targetPose) {
        setDriverAssistState(driverAssistState);
        setTargetPose(targetPose);
    }

    private DriverAssistState driverAssistState = DriverAssistState.Disabled;
    public DriverAssistState getDriverAssistState()                                     {return driverAssistState;}
    public DriverAssistCommand setDriverAssistState(DriverAssistState driverAssistState) {this.driverAssistState = driverAssistState; return this;}

    private Pose2d targetPose = new Pose2d();
    public Pose2d getTargetPose()                               {return targetPose;}
    public DriverAssistCommand setTargetPose(Pose2d targetPose) {this.targetPose = targetPose; return this;}
}
