package frc.robot.auto.actions;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveStatus;

public class DriveOnChargeStationEdgeAction extends Action {
    private static final double kUpperPitchThreshold = 20;
    private static final double kLowerPitchThreshold = 15;
    private static final double kDrivePercentOutput = 0.3;

    private final Drive drive = Drive.getInstance();
    private final DriveStatus driveStatus = DriveStatus.getInstance();

    private boolean hitUpperThreshold;

    @Override
    protected void start() {}

    @Override
    protected void run() {
        drive.setDriveCommand(new DriveCommand(kDrivePercentOutput, kDrivePercentOutput));
        if(!hitUpperThreshold) {
            if(driveStatus.getPitchDeg() >= kUpperPitchThreshold)
                hitUpperThreshold = true;
        } else {
            if(driveStatus.getPitchDeg() <= kLowerPitchThreshold)
                setFinished(true);
        }
    }

    @Override
    protected void done() {
        drive.setDriveCommand(DriveCommand.COAST());
    }
}
