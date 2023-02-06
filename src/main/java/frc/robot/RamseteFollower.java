package frc.robot;

import org.littletonrobotics.conduit.schema.SystemData;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveHAL;
import frc.robot.subsystems.odometry.OdometryStatus;

public class RamseteFollower {

    private static final double kSLinear = 1.9973;
    private static final double kVLinear = 3.0358;
    private static final double kALinear = 1.8643;
    private static final double kSAngular = 3.1514;
    private static final double kVAngular = 4.1154;
    private static final double kAAngular = 3.6761;

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
        Logger.getInstance().recordOutput("RamseteFollower/Sampled Robot Pose", trajectory.sample(lastEvalTime - startTime).poseMeters);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        Logger.getInstance().recordOutput("RamseteFollower/Wheel Speeds (M|Sec)/Left", wheelSpeeds.leftMetersPerSecond);
        Logger.getInstance().recordOutput("RamseteFollower/Wheel Speeds (M|Sec)/Right", wheelSpeeds.rightMetersPerSecond);
        DifferentialDriveWheelVoltages motorVoltages = feedForward.calculate(Units.inchesToMeters(odometry.getRobotSpeedInPerSec().left), wheelSpeeds.leftMetersPerSecond, Units.inchesToMeters(odometry.getRobotSpeedInPerSec().right), wheelSpeeds.rightMetersPerSecond, dt);
        Logger.getInstance().recordOutput("RamseteFollower/Wheel Voltages (V)/Left", motorVoltages.left);
        Logger.getInstance().recordOutput("RamseteFollower/Wheel Voltages (V)/Right", motorVoltages.right);

        double leftPower = motorVoltages.left/12;
        double rightPower = motorVoltages.right/12;
        driveCommand = new DriveCommand(leftPower, rightPower);
        return getDriveCommand();
    }

    public boolean getFinished()
    {
        double allowableTimeError = 0.01;
        return lastEvalTime - startTime >= trajectory.getTotalTimeSeconds() - allowableTimeError;
    }
}
