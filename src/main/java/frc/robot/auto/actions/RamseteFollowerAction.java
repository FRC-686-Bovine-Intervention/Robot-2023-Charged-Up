package frc.robot.auto.actions;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.RamseteFollower;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;

public class RamseteFollowerAction implements Action {
    public final RamseteFollower controller;
    private final Drive drive = Drive.getInstance();

    public RamseteFollowerAction(Trajectory path, RamseteController controller)
    {
        this.controller = new RamseteFollower(controller).setTrajectory(path);
    }

    @Override
    public void start() {
        controller.setStartTime();
    }

    @Override
    public void run() {
        drive.setDriveCommand(controller.update());
    }

    @Override
    public boolean isFinished() {
        return controller.getFinished();
    }

    @Override
    public void done() {
        drive.setDriveCommand(DriveCommand.COAST());
    }
    
}
