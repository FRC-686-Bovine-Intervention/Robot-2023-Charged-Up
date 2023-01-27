package frc.robot.subsystems.driverAssist;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveStatus;
import frc.robot.subsystems.driverAssist.DriverAssistStatus.DriverAssistState;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.odometry.OdometryStatus;

public class DriverAssistLoop extends LoopBase {
    private static DriverAssistLoop instance;
    public static DriverAssistLoop getInstance(){if(instance == null){instance = new DriverAssistLoop();}return instance;}

    private final Drive drive = Drive.getInstance();
    private final DriveStatus driveStatus = DriveStatus.getInstance();
    private final DriverAssistStatus status = DriverAssistStatus.getInstance();

    private static final double kForwardPitchThreshold = 10;
    private static final double kBackwardPitchThreshold = -10;
    private static final double kForwardBalancePercent = 0.3;
    private static final double kBackwardBalancePercent = -0.3;

    private DriverAssistLoop() {Subsystem = DriverAssist.getInstance();}

    private DriverAssistState prevState = DriverAssistState.Disabled;
    
    @Override
    public void Update() {
        DriverAssistCommand newCommand = status.getAssistCommand();

        // Determine new state
        DriverAssistState newState = DriverAssistState.Disabled;
        if(newCommand.getBalanceButton().getButton())
            newState = DriverAssistState.AutoBalance;

        status.setDriverAssistState(newState);

        // Execute new state
        switch(newState)
        {
            default:
            case Disabled:
                status.setDriveCommand(DriveCommand.COAST());
            break;
            case AutoBalance:
                double pitch = driveStatus.getPitchDeg();
                DriveCommand driveCommand = DriveCommand.BRAKE();
                if(pitch >= kForwardPitchThreshold)
                    driveCommand.setWheelSpeed(new WheelSpeeds(kForwardBalancePercent, kForwardBalancePercent));
                if(pitch <= kBackwardPitchThreshold)
                    driveCommand.setWheelSpeed(new WheelSpeeds(kBackwardBalancePercent, kBackwardBalancePercent));
                status.setDriveCommand(driveCommand);
            break;
            case AutoDrive:
                if(prevState != DriverAssistState.AutoDrive)
                {
                    ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
                    waypoints.add(OdometryStatus.getInstance().getRobotPose());
                    waypoints.add(OdometryStatus.getInstance().getRobotPose().plus(new Transform2d(new Translation2d(2,0), new Rotation2d())));
                    waypoints.add(newCommand.getTargetPose());
                    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(1.0, 2.0);
                    
                    status.setTrajectory(TrajectoryGenerator.generateTrajectory(waypoints, trajectoryConfig));
                }

                // TODO: actually set drive command
                status.setDriveCommand(DriveCommand.COAST());
            break;
        }
        // Don't override drive command if disabled
        if(newState != DriverAssistState.Disabled)
            drive.setDriveCommand(status.getDriveCommand());

        prevState = status.getDriverAssistState();
    }

    @Override public void Enabled() {}
    @Override public void Disabled() {}
}
