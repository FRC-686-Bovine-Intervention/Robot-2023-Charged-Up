package frc.robot.auto.actions;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveStatus;

public class DriveOnChargeStationEdgeAction extends Action {
    private static final double kUpperPitchThreshold = 15;
    private static final double kLowerPitchThreshold = 13;
    private static final double kDrivePercentOutput = 0.3;
    private final boolean reversed;

    private final Drive drive = Drive.getInstance();
    private final DriveStatus driveStatus = DriveStatus.getInstance();

    public DriveOnChargeStationEdgeAction() {this(false);}
    public DriveOnChargeStationEdgeAction(boolean reversed) {
        this.reversed = reversed;
    }

    private boolean hitUpperThreshold;

    @Override
    protected void start() {}

    @Override
    protected void run() {
        drive.setDriveCommand(new DriveCommand(kDrivePercentOutput * (reversed ? -1 : 1), kDrivePercentOutput * (reversed ? -1 : 1)));
        if(!hitUpperThreshold) {
            if(Math.abs(driveStatus.getPitchDeg()) >= kUpperPitchThreshold)
                hitUpperThreshold = true;
        } else {
            if(Math.abs(driveStatus.getPitchDeg()) <= kLowerPitchThreshold)
                setFinished(true);
        }
    }

    @Override
    protected void done() {
        drive.setDriveCommand(DriveCommand.COAST());
    }
}
