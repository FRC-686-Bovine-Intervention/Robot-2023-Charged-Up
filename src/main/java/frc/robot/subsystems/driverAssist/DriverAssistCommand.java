package frc.robot.subsystems.driverAssist;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.driverInteraction.DriverInteractionStatus.DriverControlButtons;

public class DriverAssistCommand {
    public DriverAssistCommand() {this(new Pose2d());}
    public DriverAssistCommand(Pose2d targetPose) {
        setTargetPose(targetPose);
    }

    private Pose2d targetPose = new Pose2d();
    public Pose2d getTargetPose()                               {return targetPose;}
    public DriverAssistCommand setTargetPose(Pose2d targetPose) {this.targetPose = targetPose; return this;}

    private DriverControlButtons balanceButton = DriverControlButtons.AutoBalance;
    public DriverControlButtons getBalanceButton()                                  {return balanceButton;}
    public DriverAssistCommand setBalanceButton(DriverControlButtons balanceButton) {this.balanceButton = balanceButton; return this;}

    private DriverControlButtons driveButton = DriverControlButtons.DriverAssist;
    public DriverControlButtons getDriveButton()                                {return driveButton;}
    public DriverAssistCommand setDriveButton(DriverControlButtons driveButton) {this.driveButton = driveButton; return this;}
}
