package frc.robot.auto.actions;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;

public class DriveStraightAction extends Action {
    private final double runTime;
    private final double setPoint;
    private final Drive drive = Drive.getInstance();

    public DriveStraightAction(double setPoint, double runTime)
    {
        this.runTime = runTime;
        this.setPoint = setPoint;
    }

    @Override
    public void start() {}

    @Override
    public void run() {
        drive.setDriveCommand(new DriveCommand(setPoint, setPoint));
        setFinished(actionTimer.hasElapsed(runTime));
    }

    @Override
    public void done() {
        drive.setDriveCommand(DriveCommand.COAST());
    }
}
