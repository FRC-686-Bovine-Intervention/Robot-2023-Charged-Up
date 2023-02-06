package frc.robot.subsystems.odometry;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveHAL;
import frc.robot.subsystems.drive.DriveStatus;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.vision.VisionStatus;

public class OdometryLoop extends LoopBase {
    private static OdometryLoop instance;
    public static OdometryLoop getInstance(){if(instance == null){instance = new OdometryLoop();}return instance;}

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DriveHAL.kTrackWidthInches));
    private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), 0, 0, new Pose2d(),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // State measurement standard deviations. X, Y, theta.
        /*new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),*/ // Local measurement standard deviations. Left encoder, right encoder, gyro.
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

    private OdometryLoop()
    {
        Subsystem = Odometry.getInstance();
    }
    
    @Override
    public void Update() {
        DriveStatus driveStatus = DriveStatus.getInstance();
        for(Pose2d poseEstimate : VisionStatus.getInstance().getVisionPoses())
        {
            poseEstimator.addVisionMeasurement(poseEstimate, Timer.getFPGATimestamp());
        }
        Pose2d poseEstimate = poseEstimator.update(driveStatus.getRotation(), Units.inchesToMeters(driveStatus.getLeftDistanceInches()), Units.inchesToMeters(driveStatus.getRightDistanceInches()));

        OdometryStatus.getInstance().setRobotPose(poseEstimate);
        OdometryStatus.getInstance().setRobotSpeedInPerSec(driveStatus.getWheelSpeeds());
    }
    
    @Override public void Enabled() {}
    @Override public void Disabled() {}
}
