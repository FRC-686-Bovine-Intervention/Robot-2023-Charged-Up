package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.FieldDimensions;
import frc.robot.util.AllianceFlipUtil;

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
                FieldDimensions.Grids.outerX + Units.inchesToMeters(Constants.kCenterToFrontBumper),
                FieldDimensions.Grids.nodeFirstY + FieldDimensions.Grids.nodeSeparationY * 1
            ),
            Rotation2d.fromDegrees(180)
        );
        Pose2d CenterScoringPose = new Pose2d(
            new Translation2d(
                FieldDimensions.Grids.outerX + Units.inchesToMeters(Constants.kCenterToFrontBumper),
                FieldDimensions.Grids.nodeFirstY + FieldDimensions.Grids.nodeSeparationY * 4
            ),
            Rotation2d.fromDegrees(180)
        );
        Pose2d LoadingScoringPose = new Pose2d(
            new Translation2d(
                FieldDimensions.Grids.outerX + Units.inchesToMeters(Constants.kCenterToFrontBumper),
                FieldDimensions.Grids.nodeFirstY + FieldDimensions.Grids.nodeSeparationY * 7
            ),
            Rotation2d.fromDegrees(180)
        );
        
        // Backup Poses
        double onePointTurnRadius = 5;
        double backupOffset = 24;

        Pose2d WallBackupPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(2 * onePointTurnRadius + Constants.kCenterToIntake + backupOffset),
                WallScoringPose.getTranslation().getY()
            ),
            Rotation2d.fromDegrees(180)
        );
        Pose2d CenterBackupPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(2 * onePointTurnRadius + Constants.kCenterToIntake + backupOffset),
                CenterScoringPose.getTranslation().getY()
            ),
            Rotation2d.fromDegrees(180)
        );
        Pose2d LoadingBackupPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(2 * onePointTurnRadius + Constants.kCenterToIntake + backupOffset),
                LoadingScoringPose.getTranslation().getY()
            ),
            Rotation2d.fromDegrees(180)
        );
        
        // Point Turn Poses
        Pose2d WallPointTurnPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(onePointTurnRadius + Constants.kCenterToIntake + backupOffset),
                FieldDimensions.StagingLocations.translations[0].getY() + Units.inchesToMeters(onePointTurnRadius)
            ),
            Rotation2d.fromDegrees(-90)
        );
        Pose2d CenterWallPointTurnPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(onePointTurnRadius + Constants.kCenterToIntake + backupOffset),
                FieldDimensions.StagingLocations.translations[1].getY() + Units.inchesToMeters(onePointTurnRadius)
            ),
            Rotation2d.fromDegrees(-90)
        );
        Pose2d CenterLoadPointTurnPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(onePointTurnRadius + Constants.kCenterToIntake + backupOffset),
                FieldDimensions.StagingLocations.translations[2].getY() - Units.inchesToMeters(onePointTurnRadius)
            ),
            Rotation2d.fromDegrees(90)
        );
        Pose2d LoadingPointTurnPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(onePointTurnRadius + Constants.kCenterToIntake + backupOffset),
                FieldDimensions.StagingLocations.translations[3].getY() + Units.inchesToMeters(onePointTurnRadius)
            ),
            Rotation2d.fromDegrees(-90)
        );
        
        // Staging Offset Poses
        Pose2d WallStagingOffsetPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(Constants.kCenterToIntake + backupOffset),
                FieldDimensions.StagingLocations.translations[0].getY()
            ),
            Rotation2d.fromDegrees(0)
        );
        Pose2d CenterWallStagingOffsetPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(Constants.kCenterToIntake + backupOffset),
                FieldDimensions.StagingLocations.translations[1].getY()
            ),
            Rotation2d.fromDegrees(0)
        );
        Pose2d CenterLoadStagingOffsetPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(Constants.kCenterToIntake + backupOffset),
                FieldDimensions.StagingLocations.translations[2].getY()
            ),
            Rotation2d.fromDegrees(0)
        );
        Pose2d LoadingStagingOffsetPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(Constants.kCenterToIntake + backupOffset),
                FieldDimensions.StagingLocations.translations[3].getY()
            ),
            Rotation2d.fromDegrees(0)
        );
        
        // Staging Poses
        Pose2d WallStagingPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(Constants.kCenterToIntake ),
                FieldDimensions.StagingLocations.translations[0].getY()
            ),
            Rotation2d.fromDegrees(0)
        );
        Pose2d CenterWallStagingPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(Constants.kCenterToIntake),
                FieldDimensions.StagingLocations.translations[1].getY()
            ),
            Rotation2d.fromDegrees(0)
        );
        Pose2d CenterLoadStagingPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(Constants.kCenterToIntake),
                FieldDimensions.StagingLocations.translations[2].getY()
            ),
            Rotation2d.fromDegrees(0)
        );
        Pose2d LoadingStagingPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(Constants.kCenterToIntake),
                FieldDimensions.StagingLocations.translations[3].getY()
            ),
            Rotation2d.fromDegrees(0)
        );

        // Charge Station Backup Pose
        Pose2d ChargeStationBackupPose = new Pose2d(
            new Translation2d(
                FieldDimensions.Community.chargingStationOuterX + Units.inchesToMeters(Constants.kCenterToFrontBumper),
                FieldDimensions.Community.chargingStationRightY + FieldDimensions.Community.chargingStationWidth
            ),
            Rotation2d.fromDegrees(0)
        );

        ArrayList<Pose2d> WallScoringBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterWallScoringBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterLoadScoringBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> LoadingScoringBackwardPath = new ArrayList<Pose2d>();

        WallScoringBackwardPath.add(WallScoringPose);
        WallScoringBackwardPath.add(WallBackupPose);
        WallScoringBackwardPath.add(WallPointTurnPose);

        CenterWallScoringBackwardPath.add(CenterScoringPose);
        CenterWallScoringBackwardPath.add(CenterBackupPose);
        CenterWallScoringBackwardPath.add(CenterWallPointTurnPose);

        CenterLoadScoringBackwardPath.add(CenterScoringPose);
        CenterLoadScoringBackwardPath.add(CenterBackupPose);
        CenterLoadScoringBackwardPath.add(CenterLoadPointTurnPose);

        LoadingScoringBackwardPath.add(LoadingScoringPose);
        LoadingScoringBackwardPath.add(LoadingBackupPose);
        LoadingScoringBackwardPath.add(LoadingPointTurnPose);

        ArrayList<Pose2d> WallPickupForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterWallPickupForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterLoadPickupForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> LoadingPickupForwardPath = new ArrayList<Pose2d>();

        WallPickupForwardPath.add(WallPointTurnPose);
        WallPickupForwardPath.add(WallStagingOffsetPose);
        WallPickupForwardPath.add(WallStagingPose);

        CenterWallPickupForwardPath.add(CenterWallPointTurnPose);
        CenterWallPickupForwardPath.add(CenterWallStagingOffsetPose);
        CenterWallPickupForwardPath.add(CenterWallStagingPose);

        CenterLoadPickupForwardPath.add(CenterLoadPointTurnPose);
        CenterLoadPickupForwardPath.add(CenterLoadStagingOffsetPose);
        CenterLoadPickupForwardPath.add(CenterLoadStagingPose);

        LoadingPickupForwardPath.add(LoadingPointTurnPose);
        LoadingPickupForwardPath.add(LoadingStagingOffsetPose);
        LoadingPickupForwardPath.add(LoadingStagingPose);

        ArrayList<Pose2d> WallPickupBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterWallPickupBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterLoadPickupBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> LoadingPickupBackwardPath = new ArrayList<Pose2d>();

        WallPickupForwardPath.add(WallStagingPose);
        WallPickupForwardPath.add(WallStagingOffsetPose);
        WallPickupForwardPath.add(WallPointTurnPose);

        CenterWallPickupForwardPath.add(CenterWallStagingPose);
        CenterWallPickupForwardPath.add(CenterWallStagingOffsetPose);
        CenterWallPickupForwardPath.add(CenterWallPointTurnPose);

        CenterLoadPickupForwardPath.add(CenterLoadStagingPose);
        CenterLoadPickupForwardPath.add(CenterLoadStagingOffsetPose);
        CenterLoadPickupForwardPath.add(CenterLoadPointTurnPose);

        LoadingPickupForwardPath.add(LoadingStagingPose);
        LoadingPickupForwardPath.add(LoadingStagingOffsetPose);
        LoadingPickupForwardPath.add(LoadingPointTurnPose);

        ArrayList<Pose2d> WallScoringForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterWallScoringForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterLoadScoringForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> LoadingScoringForwardPath = new ArrayList<Pose2d>();

        WallScoringBackwardPath.add(WallPointTurnPose);
        WallScoringBackwardPath.add(WallBackupPose);
        WallScoringBackwardPath.add(WallScoringPose);

        CenterWallScoringBackwardPath.add(CenterWallPointTurnPose);
        CenterWallScoringBackwardPath.add(CenterBackupPose);
        CenterWallScoringBackwardPath.add(CenterScoringPose);

        CenterLoadScoringBackwardPath.add(CenterLoadPointTurnPose);
        CenterLoadScoringBackwardPath.add(CenterBackupPose);
        CenterLoadScoringBackwardPath.add(CenterScoringPose);

        LoadingScoringBackwardPath.add(LoadingPointTurnPose);
        LoadingScoringBackwardPath.add(LoadingBackupPose);
        LoadingScoringBackwardPath.add(LoadingScoringPose);

        ArrayList<Pose2d> WallStationBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterWallStationBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterLoadStationBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> LoadingStationBackwardPath = new ArrayList<Pose2d>();

        WallPickupForwardPath.add(WallStagingPose);
        WallPickupForwardPath.add(ChargeStationBackupPose);

        CenterWallStationBackwardPath.add(CenterWallStagingPose);
        CenterWallStationBackwardPath.add(ChargeStationBackupPose);

        CenterLoadStationBackwardPath.add(CenterLoadStagingPose);
        CenterLoadStationBackwardPath.add(ChargeStationBackupPose);

        LoadingStationBackwardPath.add(LoadingStagingPose);
        LoadingStationBackwardPath.add(ChargeStationBackupPose);

        // Generic
        
        BlueWallScoringBackward =       TrajectoryGenerator.generateTrajectory(WallScoringBackwardPath, backwardConfig);
        BlueCenterWallScoringBackward = TrajectoryGenerator.generateTrajectory(CenterWallScoringBackwardPath, backwardConfig);
        BlueCenterLoadScoringBackward = TrajectoryGenerator.generateTrajectory(CenterLoadScoringBackwardPath, backwardConfig);
        BlueLoadingScoringBackward =    TrajectoryGenerator.generateTrajectory(LoadingScoringBackwardPath, backwardConfig);

        BlueWallPickupForward =         TrajectoryGenerator.generateTrajectory(WallPickupForwardPath, forwardConfig);
        BlueCenterWallPickupForward =   TrajectoryGenerator.generateTrajectory(CenterWallPickupForwardPath, forwardConfig);
        BlueCenterLoadPickupForward =   TrajectoryGenerator.generateTrajectory(CenterLoadPickupForwardPath, forwardConfig);
        BlueLoadingPickupForward =      TrajectoryGenerator.generateTrajectory(LoadingPickupForwardPath, forwardConfig);

        // Two Piece

        BlueWallPickupBackward =        TrajectoryGenerator.generateTrajectory(WallPickupBackwardPath, backwardConfig);
        BlueCenterWallPickupBackward =  TrajectoryGenerator.generateTrajectory(CenterWallPickupBackwardPath, backwardConfig);
        BlueCenterLoadPickupBackward =  TrajectoryGenerator.generateTrajectory(CenterLoadPickupBackwardPath, backwardConfig);
        BlueLoadingPickupBackward =     TrajectoryGenerator.generateTrajectory(LoadingPickupBackwardPath, backwardConfig);

        BlueWallScoringForward =        TrajectoryGenerator.generateTrajectory(WallScoringForwardPath, forwardConfig);
        BlueCenterWallScoringForward =  TrajectoryGenerator.generateTrajectory(CenterWallScoringForwardPath, forwardConfig);
        BlueCenterLoadScoringForward =  TrajectoryGenerator.generateTrajectory(CenterLoadScoringForwardPath, forwardConfig);
        BlueLoadingScoringForward =     TrajectoryGenerator.generateTrajectory(LoadingScoringForwardPath, forwardConfig);

        // One Piece Balance

        BlueWallStationBackward =       TrajectoryGenerator.generateTrajectory(WallStationBackwardPath, backwardConfig);
        BlueCenterWallStationBackward = TrajectoryGenerator.generateTrajectory(CenterWallStationBackwardPath, backwardConfig);
        BlueCenterLoadStationBackward = TrajectoryGenerator.generateTrajectory(CenterLoadStationBackwardPath, backwardConfig);
        BlueLoadingStationBackward =    TrajectoryGenerator.generateTrajectory(LoadingStationBackwardPath, backwardConfig);

        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);

        WallScoringBackwardPath = new ArrayList<Pose2d>();
        CenterWallScoringBackwardPath = new ArrayList<Pose2d>();
        CenterLoadScoringBackwardPath = new ArrayList<Pose2d>();
        LoadingScoringBackwardPath = new ArrayList<Pose2d>();

        WallScoringBackwardPath.add(WallScoringPose);
        WallScoringBackwardPath.add(WallBackupPose);
        WallScoringBackwardPath.add(WallPointTurnPose);

        CenterWallScoringBackwardPath.add(CenterScoringPose);
        CenterWallScoringBackwardPath.add(CenterBackupPose);
        CenterWallScoringBackwardPath.add(CenterWallPointTurnPose);

        CenterLoadScoringBackwardPath.add(CenterScoringPose);
        CenterLoadScoringBackwardPath.add(CenterBackupPose);
        CenterLoadScoringBackwardPath.add(CenterLoadPointTurnPose);

        LoadingScoringBackwardPath.add(LoadingScoringPose);
        LoadingScoringBackwardPath.add(LoadingBackupPose);
        LoadingScoringBackwardPath.add(LoadingPointTurnPose);

        WallPickupForwardPath = new ArrayList<Pose2d>();
        CenterWallPickupForwardPath = new ArrayList<Pose2d>();
        CenterLoadPickupForwardPath = new ArrayList<Pose2d>();
        LoadingPickupForwardPath = new ArrayList<Pose2d>();

        WallPickupForwardPath.add(WallPointTurnPose);
        WallPickupForwardPath.add(WallStagingOffsetPose);
        WallPickupForwardPath.add(WallStagingPose);

        CenterWallPickupForwardPath.add(CenterWallPointTurnPose);
        CenterWallPickupForwardPath.add(CenterWallStagingOffsetPose);
        CenterWallPickupForwardPath.add(CenterWallStagingPose);

        CenterLoadPickupForwardPath.add(CenterLoadPointTurnPose);
        CenterLoadPickupForwardPath.add(CenterLoadStagingOffsetPose);
        CenterLoadPickupForwardPath.add(CenterLoadStagingPose);

        LoadingPickupForwardPath.add(LoadingPointTurnPose);
        LoadingPickupForwardPath.add(LoadingStagingOffsetPose);
        LoadingPickupForwardPath.add(LoadingStagingPose);

        WallPickupBackwardPath = new ArrayList<Pose2d>();
        CenterWallPickupBackwardPath = new ArrayList<Pose2d>();
        CenterLoadPickupBackwardPath = new ArrayList<Pose2d>();
        LoadingPickupBackwardPath = new ArrayList<Pose2d>();

        WallPickupForwardPath.add(WallStagingPose);
        WallPickupForwardPath.add(WallStagingOffsetPose);
        WallPickupForwardPath.add(WallPointTurnPose);

        CenterWallPickupForwardPath.add(CenterWallStagingPose);
        CenterWallPickupForwardPath.add(CenterWallStagingOffsetPose);
        CenterWallPickupForwardPath.add(CenterWallPointTurnPose);

        CenterLoadPickupForwardPath.add(CenterLoadStagingPose);
        CenterLoadPickupForwardPath.add(CenterLoadStagingOffsetPose);
        CenterLoadPickupForwardPath.add(CenterLoadPointTurnPose);

        LoadingPickupForwardPath.add(LoadingStagingPose);
        LoadingPickupForwardPath.add(LoadingStagingOffsetPose);
        LoadingPickupForwardPath.add(LoadingPointTurnPose);

        WallScoringForwardPath = new ArrayList<Pose2d>();
        CenterWallScoringForwardPath = new ArrayList<Pose2d>();
        CenterLoadScoringForwardPath = new ArrayList<Pose2d>();
        LoadingScoringForwardPath = new ArrayList<Pose2d>();

        WallScoringBackwardPath.add(WallPointTurnPose);
        WallScoringBackwardPath.add(WallBackupPose);
        WallScoringBackwardPath.add(WallScoringPose);

        CenterWallScoringBackwardPath.add(CenterWallPointTurnPose);
        CenterWallScoringBackwardPath.add(CenterBackupPose);
        CenterWallScoringBackwardPath.add(CenterScoringPose);

        CenterLoadScoringBackwardPath.add(CenterLoadPointTurnPose);
        CenterLoadScoringBackwardPath.add(CenterBackupPose);
        CenterLoadScoringBackwardPath.add(CenterScoringPose);

        LoadingScoringBackwardPath.add(LoadingPointTurnPose);
        LoadingScoringBackwardPath.add(LoadingBackupPose);
        LoadingScoringBackwardPath.add(LoadingScoringPose);

        WallStationBackwardPath = new ArrayList<Pose2d>();
        CenterWallStationBackwardPath = new ArrayList<Pose2d>();
        CenterLoadStationBackwardPath = new ArrayList<Pose2d>();
        LoadingStationBackwardPath = new ArrayList<Pose2d>();

        WallPickupForwardPath.add(WallStagingPose);
        WallPickupForwardPath.add(ChargeStationBackupPose);

        CenterWallStationBackwardPath.add(CenterWallStagingPose);
        CenterWallStationBackwardPath.add(ChargeStationBackupPose);

        CenterLoadStationBackwardPath.add(CenterLoadStagingPose);
        CenterLoadStationBackwardPath.add(ChargeStationBackupPose);

        LoadingStationBackwardPath.add(LoadingStagingPose);
        LoadingStationBackwardPath.add(ChargeStationBackupPose);
    }
}
