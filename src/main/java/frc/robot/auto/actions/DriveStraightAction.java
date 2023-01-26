package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;

public class DriveStraightAction implements Action {
    private final double runTime;
    private final double setPoint;
    private final Drive drive = Drive.getInstance();

    public DriveStraightAction(double setPoint, double runTime)
    {
        this.runTime = runTime;
        this.setPoint = setPoint;
    }

    private double startTime;
    
    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void run() {
        drive.setDriveCommand(new DriveCommand(setPoint, setPoint));
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= runTime; // this part doesnt work the best
    }

    @Override
    public void done() {
        drive.setDriveCommand(DriveCommand.COAST());
    }
    
}
