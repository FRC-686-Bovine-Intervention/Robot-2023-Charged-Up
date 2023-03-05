package frc.robot;

import org.littletonrobotics.junction.Logger;

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

    private static final double kSLinear =  0.3032;
    private static final double kVLinear =  2.662;
    private static final double kALinear =  0.9436;
    private static final double kSAngular = 1.0076;
    private static final double kVAngular = 2.6471;
    private static final double kAAngular = 1.0362;

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
        Logger.getInstance().recordOutput("RamseteFollower/dt", dt);
        ChassisSpeeds chassisSpeeds = controller.calculate(odometry.getRobotPose(), trajectory.sample(lastEvalTime - startTime));
        Logger.getInstance().recordOutput("RamseteFollower/Sampled Robot Pose", trajectory.sample(lastEvalTime - startTime).poseMeters);
        Logger.getInstance().recordOutput("RamseteFollower/Chassis Speeds/Linear (M|Sec)", chassisSpeeds.vxMetersPerSecond);
        Logger.getInstance().recordOutput("RamseteFollower/Chassis Speeds/Angular (Rad|Sec)", chassisSpeeds.omegaRadiansPerSecond);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        wheelSpeeds.leftMetersPerSecond     = 0.2;
        wheelSpeeds.rightMetersPerSecond    = 0.2;
        Logger.getInstance().recordOutput("RamseteFollower/Wheel Speeds (M|Sec)/Left", wheelSpeeds.leftMetersPerSecond);
        Logger.getInstance().recordOutput("RamseteFollower/Wheel Speeds (M|Sec)/Right", wheelSpeeds.rightMetersPerSecond);
        DifferentialDriveWheelVoltages motorVoltages = feedForward.calculate(odometry.getRobotSpeedMeterPerSec().left, wheelSpeeds.leftMetersPerSecond, odometry.getRobotSpeedMeterPerSec().right, wheelSpeeds.rightMetersPerSecond, dt);
        Logger.getInstance().recordOutput("RamseteFollower/Wheel Voltages (V)/Left", motorVoltages.left);
        Logger.getInstance().recordOutput("RamseteFollower/Wheel Voltages (V)/Right", motorVoltages.right);

        double leftPower = motorVoltages.left/12;
        double rightPower = motorVoltages.right/12;
        driveCommand = new DriveCommand(leftPower, rightPower);
        return getDriveCommand();
    }

    public boolean getFinished()
    {
        return false;
        // double allowableTimeError = 0.01;
        // return lastEvalTime - startTime >= trajectory.getTotalTimeSeconds() - allowableTimeError;
    }
}
