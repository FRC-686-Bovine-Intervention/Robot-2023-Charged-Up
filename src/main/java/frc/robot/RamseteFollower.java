package frc.robot;

import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveHAL;
import frc.robot.subsystems.odometry.OdometryStatus;

public class RamseteFollower {

    private static final double kSLinear = 1.6871;
    private static final double kVLinear = 3.0149;
    private static final double kALinear = 5.6755;
    private static final double kVAngular = 0.05;
    private static final double kAAngular = 0.05;

    private final RamseteController controller;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DriveHAL.kTrackWidthInches));
    private final DifferentialDriveFeedforward feedForward = new DifferentialDriveFeedforward(kVLinear,kALinear,kVAngular,kAAngular,Units.inchesToMeters(DriveHAL.kTrackWidthInches));
    private final OdometryStatus odometry = OdometryStatus.getInstance();

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
    public RamseteFollower setStartTime() {return setStartTime(Timer.getFPGATimestamp());}
    public RamseteFollower setStartTime(double startTime) {this.startTime = startTime; return this;}
    
    private double lastEvalTime;
    public double getLastEvalTime() {return lastEvalTime;}

    public DriveCommand update() {return update(Timer.getFPGATimestamp() - lastEvalTime);}
    public DriveCommand update(double dt)
    {
        lastEvalTime += dt;
        ChassisSpeeds chassisSpeeds = controller.calculate(odometry.getRobotPose(), trajectory.sample(lastEvalTime - startTime));
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        DifferentialDriveWheelVoltages motorVoltages = feedForward.calculate(odometry.getRobotSpeed().left, wheelSpeeds.leftMetersPerSecond, odometry.getRobotSpeed().right, wheelSpeeds.rightMetersPerSecond, dt);

        driveCommand = new DriveCommand(motorVoltages.left, motorVoltages.right);
        return getDriveCommand();
    }

    public boolean getFinished()
    {
        double allowableTimeError = 0.01;
        return lastEvalTime - startTime >= trajectory.getTotalTimeSeconds() - allowableTimeError;
    }
}
