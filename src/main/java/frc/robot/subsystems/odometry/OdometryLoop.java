package frc.robot.subsystems.odometry;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveHAL;
import frc.robot.subsystems.drive.DriveStatus;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.vision.VisionStatus;
import frc.robot.subsystems.vision.VisionStatus.VisionData;

public class OdometryLoop extends LoopBase {
    private static OdometryLoop instance;
    public static OdometryLoop getInstance(){if(instance == null){instance = new OdometryLoop();}return instance;}

    private final OdometryStatus status = OdometryStatus.getInstance();
    private final DriveStatus driveStatus = DriveStatus.getInstance();
    private final VisionStatus visionStatus = VisionStatus.getInstance();

    private static final Translation3d camOrigin = new Translation3d();
    private static final double kMaxTrustedCamDistance = 1.5;

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DriveHAL.kTrackWidthInches));
    private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(180), 0, 0, new Pose2d(),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Drivetrain measurement standard deviations. X, Y, theta.
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.2, 0.2, 0.2)); // Vision measurement standard deviations. X, Y, and theta.

    private OdometryLoop() {Subsystem = Odometry.getInstance();}
    
    @Override
    public void Update() {
        OdometryCommand newCommand = status.getCommand();

        Pose2d poseEstimate = 
            poseEstimator.update(
                driveStatus.getRotation(), 
                Units.inchesToMeters(driveStatus.getLeftDistanceInches()), 
                Units.inchesToMeters(driveStatus.getRightDistanceInches())
            );

        // for(EstimatedRobotPose data : visionStatus.getVisionData()) {
        //     boolean goodData = false;
        //     for (PhotonTrackedTarget targetsUsed : data.targetsUsed) {
        //         if(camOrigin.getDistance(targetsUsed.getBestCameraToTarget().getTranslation()) <= kMaxTrustedCamDistance) {
        //             goodData = true;
        //             break;
        //         }
        //     }
        //     if(goodData)
        //         poseEstimator.addVisionMeasurement(data.estimatedPose.toPose2d(), data.timestampSeconds);
        // }
        
        for(EstimatedRobotPose data : visionStatus.getVisionData()) {
            if(data.targetsUsed.size() > 1 || (data.targetsUsed.size() == 1 && data.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getDistance(camOrigin) <= kMaxTrustedCamDistance))
                poseEstimator.addVisionMeasurement(data.estimatedPose.toPose2d(), data.timestampSeconds/* , data.getStdDevs() */);
        }

        if(newCommand.getResetPose() != null) {
            poseEstimate = newCommand.getResetPose();
            poseEstimator.resetPosition(
                driveStatus.getRotation(), 
                Units.inchesToMeters(driveStatus.getLeftDistanceInches()), 
                Units.inchesToMeters(driveStatus.getRightDistanceInches()), 
                poseEstimate
            );
        }

        status.setRobotPose(poseEstimate);
        status.setRobotSpeedInPerSec(driveStatus.getWheelSpeeds());
    }
    
    @Override public void Enabled() {}
    @Override public void Disabled() {}
}
