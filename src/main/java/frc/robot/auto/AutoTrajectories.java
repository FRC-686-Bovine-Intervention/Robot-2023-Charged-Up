package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldDimensions;

public class AutoTrajectories {
    // Generic
    public static final Trajectory RedWallScoringBackward;
    public static final Trajectory RedCenterWallScoringBackward;
    public static final Trajectory RedCenterLoadScoringBackward;
    public static final Trajectory RedLoadingScoringBackward;
    public static final Trajectory RedWallPickupForward;
    public static final Trajectory RedCenterWallPickupForward;
    public static final Trajectory RedCenterLoadPickupForward;
    public static final Trajectory RedLoadingPickupForward;
    public static final Trajectory BlueWallScoringBackward;
    public static final Trajectory BlueCenterWallScoringBackward;
    public static final Trajectory BlueCenterLoadScoringBackward;
    public static final Trajectory BlueLoadingScoringBackward;
    public static final Trajectory BlueWallPickupForward;
    public static final Trajectory BlueCenterWallPickupForward;
    public static final Trajectory BlueCenterLoadPickupForward;
    public static final Trajectory BlueLoadingPickupForward;

    // Two Piece
    public static final Trajectory RedWallPickupBackward;
    public static final Trajectory RedCenterWallPickupBackward;
    public static final Trajectory RedCenterLoadPickupBackward;
    public static final Trajectory RedLoadingPickupBackward;
    public static final Trajectory RedWallScoringForward;
    public static final Trajectory RedCenterWallScoringForward;
    public static final Trajectory RedCenterLoadScoringForward;
    public static final Trajectory RedLoadingScoringForward;
    public static final Trajectory BlueWallPickupBackward;
    public static final Trajectory BlueCenterWallPickupBackward;
    public static final Trajectory BlueCenterLoadPickupBackward;
    public static final Trajectory BlueLoadingPickupBackward;
    public static final Trajectory BlueWallScoringForward;
    public static final Trajectory BlueCenterWallScoringForward;
    public static final Trajectory BlueCenterLoadScoringForward;
    public static final Trajectory BlueLoadingScoringForward;
    
    // One Piece Balance
    public static final Trajectory RedWallStationBackward;
    public static final Trajectory RedCenterWallStationBackward;
    public static final Trajectory RedCenterLoadStationBackward;
    public static final Trajectory RedLoadingStationBackward;
    public static final Trajectory BlueWallStationBackward;
    public static final Trajectory BlueCenterWallStationBackward;
    public static final Trajectory BlueCenterLoadStationBackward;
    public static final Trajectory BlueLoadingStationBackward;

    static {
        double kMaxVelocityMPS =      Units.inchesToMeters(48);
        double kMaxAccelerationMPSS = kMaxVelocityMPS * 2;

        TrajectoryConfig forwardConfig = new TrajectoryConfig(kMaxVelocityMPS, kMaxAccelerationMPSS);
        TrajectoryConfig backwardConfig = new TrajectoryConfig(kMaxVelocityMPS, kMaxAccelerationMPSS);
        backwardConfig.setReversed(true);

        // Scoring Poses
        Pose2d WallScoringPose = new Pose2d(
            new Translation2d(
                FieldDimensions.Grids.outerX + Constants.kCenterToFrontBumper,
                FieldDimensions.Grids.nodeFirstY + FieldDimensions.Grids.nodeSeparationY * 1
            ),
            Rotation2d.fromDegrees(180)
        );
        Pose2d CenterScoringPose = new Pose2d(
            new Translation2d(
                FieldDimensions.Grids.outerX + Constants.kCenterToFrontBumper,
                FieldDimensions.Grids.nodeFirstY + FieldDimensions.Grids.nodeSeparationY * 4
            ),
            Rotation2d.fromDegrees(180)
        );
        Pose2d LoadingScoringPose = new Pose2d(
            new Translation2d(
                FieldDimensions.Grids.outerX + Constants.kCenterToFrontBumper,
                FieldDimensions.Grids.nodeFirstY + FieldDimensions.Grids.nodeSeparationY * 7
            ),
            Rotation2d.fromDegrees(180)
        );
        
        // Backup Poses
        double onePointTurnRadius = 5;


        Pose2d WallBackupPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.translations[3].getX() - Units.inchesToMeters(2 * onePointTurnRadius + Constants.kCenterToIntake),
                WallScoringPose.getTranslation().getY()
            ),
            Rotation2d.fromDegrees(180)
        );
        Pose2d CenterScoringPose = new Pose2d(
            new Translation2d(
                FieldDimensions.Grids.outerX + Constants.kCenterToFrontBumper,
                FieldDimensions.Grids.nodeFirstY + FieldDimensions.Grids.nodeSeparationY * 4
            ),
            Rotation2d.fromDegrees(180)
        );
        Pose2d LoadingScoringPose = new Pose2d(
            new Translation2d(
                FieldDimensions.Grids.outerX + Constants.kCenterToFrontBumper,
                FieldDimensions.Grids.nodeFirstY + FieldDimensions.Grids.nodeSeparationY * 7
            ),
            Rotation2d.fromDegrees(180)
        );
        
    }
}
