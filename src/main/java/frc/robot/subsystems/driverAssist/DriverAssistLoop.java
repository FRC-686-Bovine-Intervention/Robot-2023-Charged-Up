package frc.robot.subsystems.driverAssist;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
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

    private static final double kForwardPitchThreshold  = 10;
    private static final double kBackwardPitchThreshold = -10;
    private static final double kPitchSpeedThreshold    = 10;
    private static final double kForwardBalancePercent  = 0.28;
    private static final double kBackwardBalancePercent = -0.28;

    private DriverAssistLoop() {Subsystem = DriverAssist.getInstance();}

    private DriverAssistState prevState = DriverAssistState.Disabled;

    private double prevPitch = 0;
    private double prevLoopTimestamp = 0;
    private double startEncoderDist = 0;
    
    @Override
    public void Update() {
        DriverAssistCommand newCommand = status.getAssistCommand();
        double currentTimestamp = Timer.getFPGATimestamp();

        // Determine new state
        status.setDriverAssistState(newCommand.getDriverAssistState());

        // Execute new state

        double dt = currentTimestamp - prevLoopTimestamp;
        double pitchVelocity = (driveStatus.getPitchDeg() - prevPitch) / dt;
        double estimatedPitch = driveStatus.getPitchDeg() + pitchVelocity * dt;

        status.setEstimatedPitch(estimatedPitch);
        status.setPitchVelo(pitchVelocity);

        switch(status.getDriverAssistState())
        {
            default:
            case Disabled:
                status.setDriveCommand(DriveCommand.COAST());
            break;

            case AutoBalance:
                double averageEncoderDist = (driveStatus.getLeftDistanceInches() + driveStatus.getRightDistanceInches()) / 2;
                if(prevState != DriverAssistState.AutoBalance)
                {
                    status.setUsingProportional(false);
                    startEncoderDist = averageEncoderDist;
                }
                DriveCommand driveCommand = DriveCommand.BRAKE();
                double output = 0;
                if(status.getUsingProportional())
                {
                    output = Math.max(Math.min(estimatedPitch*0.25/15, 0.25),-0.25);
                }
                else
                {
                    output = Math.signum(driveStatus.getPitchDeg()) * 0.25;
                    // if(Math.signum(driveStatus.getPitchDeg()) != Math.signum(prevPitch))
                    if(Math.abs(averageEncoderDist - startEncoderDist) >= 24 + 15)
                        status.setUsingProportional(true);
                }
                driveCommand.setWheelSpeed(new WheelSpeeds(output,output));
                // if(estimatedPitch >= kForwardPitchThreshold)
                //     driveCommand.setWheelSpeed(new WheelSpeeds(kForwardBalancePercent, kForwardBalancePercent));
                // if(estimatedPitch <= kBackwardPitchThreshold)
                //     driveCommand.setWheelSpeed(new WheelSpeeds(kBackwardBalancePercent, kBackwardBalancePercent));
                // if(Math.abs(pitchVelocity) >= kPitchSpeedThreshold)
                //     driveCommand.setWheelSpeed(new WheelSpeeds(0,0));
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
        if(status.getDriverAssistState() != DriverAssistState.Disabled)
            drive.setDriveCommand(status.getDriveCommand());
        
        prevPitch = driveStatus.getPitchDeg();
        prevLoopTimestamp = currentTimestamp;
        prevState = status.getDriverAssistState();
    }

    @Override public void Enabled() {}
    @Override public void Disabled() {}
}
