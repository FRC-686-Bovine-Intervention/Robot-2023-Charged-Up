package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.drive.DriveCommand;

public class RamseteFollower {
    private final RamseteController controller;
    private Trajectory trajectory;

    public RamseteFollower(double b, double zeta)
    {
        controller = new RamseteController(b, zeta);
    }

    public RamseteFollower setTrajectory(Trajectory trajectory) {this.trajectory = trajectory; return this;}
    public Trajectory getTrajectory() {return trajectory;}

    public DriveCommand getDriveCommand()
    {
        return new DriveCommand(null, null);
    }
}
