package frc.robot.auto.actions;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveStatus;

public class DrivePercentAction extends Action {
    private final double distance;
    private final double setPoint;
    private final Drive drive = Drive.getInstance();
    private final DriveStatus driveStatus = DriveStatus.getInstance();

    public DrivePercentAction(double distance, double setPoint)
    {
        this.distance = distance;
        this.setPoint = setPoint;
    }

    private double startDistance;

    @Override
    public void start() {
        startDistance = getDistance();
    }

    @Override
    public void run() {
        drive.setDriveCommand(new DriveCommand(setPoint, setPoint));
        setFinished(Math.abs(getDistance() - startDistance) >= distance);
    }

    @Override
    public void done() {
        drive.setDriveCommand(DriveCommand.COAST());
    }

    private double getDistance() {return (driveStatus.getLeftDistanceInches() + driveStatus.getRightDistanceInches()) / 2;}
}
