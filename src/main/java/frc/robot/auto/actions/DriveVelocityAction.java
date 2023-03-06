package frc.robot.auto.actions;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveStatus;
import frc.robot.subsystems.drive.DriveCommand.DriveControlMode;

public class DriveVelocityAction extends Action {
    private final Drive drive = Drive.getInstance();
    private final DriveStatus driveStatus = DriveStatus.getInstance();
    private final double targetDistance;
    private final double velocity;

    public DriveVelocityAction(double targetDistance, double velocity) {
        this.targetDistance = targetDistance;
        this.velocity = velocity;
    }

    private double startingDistance;

    @Override
    protected void start() {
        startingDistance = getCurrentDistance();
    }

    @Override
    protected void run() {
        drive.setDriveCommand(new DriveCommand(DriveControlMode.VELOCITY_SETPOINT, velocity, velocity));
        setFinished(getCurrentDistance() - startingDistance >= targetDistance);
    }

    @Override
    protected void done() {
        drive.setDriveCommand(DriveCommand.COAST());
    }

    private double getCurrentDistance() {
        return (driveStatus.getLeftDistanceInches() + driveStatus.getRightDistanceInches()) / 2;
    }
}
