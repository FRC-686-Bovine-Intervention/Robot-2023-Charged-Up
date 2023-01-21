package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveCommand.DriveControlMode;
import frc.robot.subsystems.drive.DriveHAL;
import frc.robot.subsystems.odometry.OdometryStatus;

public class RamseteFollower {
    private final RamseteController controller;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DriveHAL.kTrackWidthInches));

    public RamseteFollower(double b, double zeta) {this(new RamseteController(b, zeta));}
    public RamseteFollower(RamseteController controller)
    {
        this.controller = controller;
        setStartTime(Timer.getFPGATimestamp());
        lastEvalTime = startTime;
    }

    private Trajectory trajectory;
    public Trajectory getTrajectory() {return trajectory;}
    public RamseteFollower setTrajectory(Trajectory trajectory) {this.trajectory = trajectory; return this;}

    private DriveCommand driveCommand;
    public DriveCommand getDriveCommand() {return driveCommand;}
    
    private double startTime;
    public double getStartTime() {return startTime;}
    public RamseteFollower setStartTime(double startTime) {this.startTime = startTime; return this;}
    
    private double lastEvalTime;
    public double getLastEvalTime() {return lastEvalTime;}

    public DriveCommand update() {return update(Timer.getFPGATimestamp() - lastEvalTime);}
    public DriveCommand update(double dt)
    {
        lastEvalTime += dt;
        ChassisSpeeds chassisSpeeds = controller.calculate(OdometryStatus.getInstance().getRobotPose(), trajectory.sample(lastEvalTime - startTime));

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        driveCommand = new DriveCommand(DriveControlMode.VELOCITY_SETPOINT, Units.metersToInches(wheelSpeeds.leftMetersPerSecond), Units.metersToInches(wheelSpeeds.rightMetersPerSecond));
        return getDriveCommand();
    }

    public boolean getFinished()
    {
        double allowableTimeError = 0.2;
        return lastEvalTime - startTime - allowableTimeError >= trajectory.getTotalTimeSeconds();
    }
}
