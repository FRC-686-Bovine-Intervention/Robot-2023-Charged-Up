package frc.robot.subsystems.driverAssist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.robot.RamseteFollower;
import frc.robot.auto.autoManager.AutoConfiguration.GamePiece;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveStatus;
import frc.robot.subsystems.driverAssist.DriverAssistStatus.DriverAssistState;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionCommand;
import frc.robot.subsystems.vision.VisionStatus;
import frc.robot.subsystems.vision.VisionStatus.LimelightPipeline;

public class DriverAssistLoop extends LoopBase {
    private static DriverAssistLoop instance;
    public static DriverAssistLoop getInstance(){if(instance == null){instance = new DriverAssistLoop();}return instance;}

    private final Drive drive = Drive.getInstance();
    private final DriveStatus driveStatus = DriveStatus.getInstance();
    private final DriverAssistStatus status = DriverAssistStatus.getInstance();

    private final Vision vision = Vision.getInstance();
    private final VisionStatus visionStatus = VisionStatus.getInstance();

    private static final double kPowerAtMaxPitch = 0.25*0.65;
    private static final double kMaxPitch = 15;

    private DriverAssistLoop() {Subsystem = DriverAssist.getInstance();}

    private DriverAssistState prevState = DriverAssistState.Disabled;

    private double prevPitch = 0;
    private double prevLoopTimestamp = 0;
    private double startEncoderDist = 0;

    private final RamseteController ramseteController = new RamseteController(2, 0.7);
    private RamseteFollower ramseteFollower;
    
    @Override
    public void Update() {
        DriverAssistCommand newCommand = status.getAssistCommand();
        double currentTimestamp = Timer.getFPGATimestamp();

        // Determine new state
        if(newCommand.getDriverAssistState() != null) {
            if(newCommand.getDriverAssistState() != DriverAssistState.AutoIntake && status.getDriverAssistState() == DriverAssistState.AutoIntake) {
                status.setTargetPiece(null)
                      .setTargetBelowCamera(false);
            }
            status.setDriverAssistState(newCommand.getDriverAssistState());
        }
        if(newCommand.getTargetGamePiece() != null)
            status.setTargetPiece(newCommand.getTargetGamePiece());

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
                // if(status.getUsingProportional())
                // {
                    output = Math.max(Math.min(estimatedPitch*kPowerAtMaxPitch/kMaxPitch, kPowerAtMaxPitch),-kPowerAtMaxPitch);
                // }
                // else
                // {
                //     output = Math.signum(driveStatus.getPitchDeg()) * 0.25;
                //     // if(Math.signum(driveStatus.getPitchDeg()) != Math.signum(prevPitch))
                //     if(Math.abs(averageEncoderDist - startEncoderDist) >= 15)
                //         status.setUsingProportional(true);
                // }
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
                // if(prevState != DriverAssistState.AutoDrive)
                // {
                //     ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
                //     waypoints.add(OdometryStatus.getInstance().getRobotPose());
                //     waypoints.add(OdometryStatus.getInstance().getRobotPose().plus(new Transform2d(new Translation2d(2,0), new Rotation2d())));
                //     waypoints.add(newCommand.getTargetPose());
                //     TrajectoryConfig trajectoryConfig = new TrajectoryConfig(1.0, 2.0);
                    
                //     status.setTrajectory(TrajectoryGenerator.generateTrajectory(waypoints, trajectoryConfig));

                //     ramseteFollower = new RamseteFollower(ramseteController).setTrajectory(status.getTrajectory());
                // }

                // // TODO: actually set drive command
                // status.setDriveCommand(ramseteFollower.getDriveCommand());
            break;

            case AutoIntake:
                double xOff = -686;
                if(status.getTargetPiece() == null) {
                    if(visionStatus.getCurrentPipeline() == LimelightPipeline.Cone && visionStatus.getTargetExists()) {
                        status.setTargetPiece(GamePiece.Cone);
                    } else {
                        if(visionStatus.getCurrentPipeline() == LimelightPipeline.Cube && visionStatus.getTargetExists()) {
                            status.setTargetPiece(GamePiece.Cube);
                        } else {
                            LimelightPipeline pipeline;
                            if(visionStatus.getCurrentPipeline() == LimelightPipeline.Cone) {
                                pipeline = LimelightPipeline.Cube;
                            } else {
                                pipeline = LimelightPipeline.Cone;
                            }
                            vision.setVisionCommand(new VisionCommand(pipeline));
                        }
                    }
                } else {
                    switch(status.getTargetPiece()) {
                        case Cone:
                            vision.setVisionCommand(new VisionCommand(LimelightPipeline.Cone));
                            if(visionStatus.getCurrentPipeline() != LimelightPipeline.Cone)
                                break;
                            if(visionStatus.getTargetExists()) {
                                if(visionStatus.getTargetXAngle() > 0) {
                                    status.setTargetBelowCamera(true);
                                }
                                if(!status.getTargetBelowCamera() || visionStatus.getTargetXAngle() > 0) {
                                    xOff = visionStatus.getTargetYAngle();
                                }
                            }
                        break;
                        case Cube:
                            vision.setVisionCommand(new VisionCommand(LimelightPipeline.Cube));
                            if(visionStatus.getCurrentPipeline() != LimelightPipeline.Cube)
                                break;
                            if(visionStatus.getTargetExists()) {
                                if(visionStatus.getTargetXAngle() > 0) {
                                    status.setTargetBelowCamera(true);
                                }
                                if(!status.getTargetBelowCamera() || visionStatus.getTargetXAngle() > 0) {
                                    xOff = visionStatus.getTargetYAngle();
                                }
                            }
                        break;
                    }
                }
                double threshold = 2.5;
                double speed = drive.getDriveCommand().getSpeed();
                double kp = 0.2/20;
                if(xOff != -686) {
                    if(Math.abs(xOff) < threshold) {
                        xOff = 0;
                    }
                    status.setDriveCommand(new DriveCommand(new WheelSpeeds(speed + xOff * kp, speed - xOff * kp)));
                } else {
                    status.setDriveCommand(drive.getDriveCommand());
                }
                Logger.getInstance().recordOutput("DEBUG/xOff", xOff);
                Logger.getInstance().recordOutput("DEBUG/speed", speed);
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
