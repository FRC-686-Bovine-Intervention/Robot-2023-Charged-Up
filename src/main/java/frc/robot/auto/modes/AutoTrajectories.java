package frc.robot.auto.modes;

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
import frc.robot.auto.autoManager.AutoConfiguration;
import frc.robot.util.AllianceFlipUtil;

public class AutoTrajectories {
    // Generic
    public static final Trajectory[][] ScoringBackward;
    public static final Trajectory[][] PickupForward;

    // One Skip
    public static final Trajectory[][] SkipBackward;

    // Two Piece
    public static final Trajectory[][] PickupBackward;
    public static final Trajectory[][] ScoringForward;

    public static final Trajectory[][] TwoPieceForward;
    public static final Trajectory[][] TwoPieceBackward;

    // One Piece Balance
    public static final Trajectory[][] StationBackward;

    static {
        double kMaxVelocityMPS =      Units.inchesToMeters(48);
        double kMaxAccelerationMPSS = kMaxVelocityMPS / 1;

        TrajectoryConfig forwardConfig = new TrajectoryConfig(kMaxVelocityMPS, kMaxAccelerationMPSS);
        TrajectoryConfig backwardConfig = new TrajectoryConfig(kMaxVelocityMPS, kMaxAccelerationMPSS);
        backwardConfig.setReversed(true);

        // Scoring Poses
        double outsideScoringOffset = FieldDimensions.Grids.nodeSeparationY / 2;

        Pose2d WallScoringPose = new Pose2d(
            new Translation2d(
                FieldDimensions.Grids.outerX + Units.inchesToMeters(Constants.kCenterToFrontBumper),
                FieldDimensions.Grids.nodeFirstY + FieldDimensions.Grids.nodeSeparationY * 1 - outsideScoringOffset
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
                FieldDimensions.Grids.nodeFirstY + FieldDimensions.Grids.nodeSeparationY * 7 + outsideScoringOffset
            ),
            Rotation2d.fromDegrees(180)
        );
        Pose2d TwoPieceWallScoringPose = new Pose2d(
            WallScoringPose.getTranslation(),
            Rotation2d.fromDegrees(0)
        );
        Pose2d TwoPieceCenterScoringPose = new Pose2d(
            CenterScoringPose.getTranslation(),
            Rotation2d.fromDegrees(0)
        );
        Pose2d TwoPieceLoadScoringPose = new Pose2d(
            LoadingScoringPose.getTranslation(),
            Rotation2d.fromDegrees(0)
        );

        // Backup Poses
        double onePointTurnRadius = 18;
        double onePointTurnCenterOffset = 18;
        double backupOffset = 18;
        double uTurnOffset = 36;

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
        Pose2d CenterWallUTurnPose = new Pose2d(
            new Translation2d(
                CenterBackupPose.getTranslation().getX() + Units.inchesToMeters(uTurnOffset),
                CenterBackupPose.getY()
            ),
            Rotation2d.fromDegrees(90)
        );
        Pose2d CenterLoadUTurnPose = new Pose2d(
            new Translation2d(
                CenterBackupPose.getTranslation().getX() + Units.inchesToMeters(uTurnOffset),
                CenterBackupPose.getY()
            ),
            Rotation2d.fromDegrees(-90)
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
                FieldDimensions.StagingLocations.translations[1].getY() + Units.inchesToMeters(onePointTurnRadius + onePointTurnCenterOffset)
            ),
            Rotation2d.fromDegrees(-90)
        );
        Pose2d CenterLoadPointTurnPose = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(onePointTurnRadius + Constants.kCenterToIntake + backupOffset),
                FieldDimensions.StagingLocations.translations[2].getY() - Units.inchesToMeters(onePointTurnRadius + onePointTurnCenterOffset)
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
                FieldDimensions.StagingLocations.positionX - Units.inchesToMeters(Constants.kCenterToIntake),
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
                FieldDimensions.Community.chargingStationRightY + FieldDimensions.Community.chargingStationWidth / 2
            ),
            Rotation2d.fromDegrees(0)
        );

        ArrayList<Pose2d> TwoPieceWallForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> TwoPieceCenterWallForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> TwoPieceCenterLoadForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> TwoPieceLoadForwardPath = new ArrayList<Pose2d>();

        TwoPieceWallForwardPath.add(TwoPieceWallScoringPose);
        TwoPieceWallForwardPath.add(WallStagingPose);

        TwoPieceCenterWallForwardPath.add(TwoPieceCenterScoringPose);
        TwoPieceCenterWallForwardPath.add(CenterWallStagingPose);
        
        TwoPieceCenterLoadForwardPath.add(TwoPieceCenterScoringPose);
        TwoPieceCenterLoadForwardPath.add(CenterLoadStagingPose);
        
        TwoPieceLoadForwardPath.add(TwoPieceLoadScoringPose);
        TwoPieceLoadForwardPath.add(LoadingStagingPose);

        ArrayList<Pose2d> TwoPieceWallBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> TwoPieceCenterWallBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> TwoPieceCenterLoadBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> TwoPieceLoadBackwardPath = new ArrayList<Pose2d>();

        TwoPieceWallBackwardPath.add(WallStagingPose);
        TwoPieceWallBackwardPath.add(TwoPieceWallScoringPose);

        TwoPieceCenterWallBackwardPath.add(CenterWallStagingPose);
        TwoPieceCenterWallBackwardPath.add(TwoPieceCenterScoringPose);
        
        TwoPieceCenterLoadBackwardPath.add(CenterLoadStagingPose);
        TwoPieceCenterLoadBackwardPath.add(TwoPieceCenterScoringPose);
        
        TwoPieceLoadBackwardPath.add(LoadingStagingPose);
        TwoPieceLoadBackwardPath.add(TwoPieceLoadScoringPose);

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

        ArrayList<Pose2d> WallSkipBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterWallSkipBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterLoadSkipBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> LoadingSkipBackwardPath = new ArrayList<Pose2d>();

        WallSkipBackwardPath.add(WallScoringPose);
        WallSkipBackwardPath.add(WallBackupPose);
        WallSkipBackwardPath.add(ChargeStationBackupPose);

        CenterWallSkipBackwardPath.add(CenterScoringPose);
        CenterWallSkipBackwardPath.add(CenterBackupPose);
        CenterWallSkipBackwardPath.add(CenterWallUTurnPose);
        CenterWallSkipBackwardPath.add(ChargeStationBackupPose);
        
        CenterLoadSkipBackwardPath.add(CenterScoringPose);
        CenterLoadSkipBackwardPath.add(CenterBackupPose);
        CenterLoadSkipBackwardPath.add(CenterLoadUTurnPose);
        CenterLoadSkipBackwardPath.add(ChargeStationBackupPose);

        LoadingSkipBackwardPath.add(LoadingScoringPose);
        LoadingSkipBackwardPath.add(LoadingBackupPose);
        LoadingSkipBackwardPath.add(ChargeStationBackupPose);

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

        WallPickupBackwardPath.add(WallStagingPose);
        WallPickupBackwardPath.add(WallStagingOffsetPose);
        WallPickupBackwardPath.add(WallPointTurnPose);

        CenterWallPickupBackwardPath.add(CenterWallStagingPose);
        CenterWallPickupBackwardPath.add(CenterWallStagingOffsetPose);
        CenterWallPickupBackwardPath.add(CenterWallPointTurnPose);

        CenterLoadPickupBackwardPath.add(CenterLoadStagingPose);
        CenterLoadPickupBackwardPath.add(CenterLoadStagingOffsetPose);
        CenterLoadPickupBackwardPath.add(CenterLoadPointTurnPose);

        LoadingPickupBackwardPath.add(LoadingStagingPose);
        LoadingPickupBackwardPath.add(LoadingStagingOffsetPose);
        LoadingPickupBackwardPath.add(LoadingPointTurnPose);

        ArrayList<Pose2d> WallScoringForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterWallScoringForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterLoadScoringForwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> LoadingScoringForwardPath = new ArrayList<Pose2d>();

        WallScoringForwardPath.add(WallPointTurnPose);
        WallScoringForwardPath.add(WallBackupPose);
        WallScoringForwardPath.add(WallScoringPose);

        CenterWallScoringForwardPath.add(CenterWallPointTurnPose);
        CenterWallScoringForwardPath.add(CenterBackupPose);
        CenterWallScoringForwardPath.add(CenterScoringPose);

        CenterLoadScoringForwardPath.add(CenterLoadPointTurnPose);
        CenterLoadScoringForwardPath.add(CenterBackupPose);
        CenterLoadScoringForwardPath.add(CenterScoringPose);

        LoadingScoringForwardPath.add(LoadingPointTurnPose);
        LoadingScoringForwardPath.add(LoadingBackupPose);
        LoadingScoringForwardPath.add(LoadingScoringPose);

        ArrayList<Pose2d> WallStationBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterWallStationBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> CenterLoadStationBackwardPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> LoadingStationBackwardPath = new ArrayList<Pose2d>();

        WallStationBackwardPath.add(WallStagingPose);
        WallStationBackwardPath.add(ChargeStationBackupPose);

        CenterWallStationBackwardPath.add(CenterWallStagingPose);
        CenterWallStationBackwardPath.add(ChargeStationBackupPose);

        CenterLoadStationBackwardPath.add(CenterLoadStagingPose);
        CenterLoadStationBackwardPath.add(ChargeStationBackupPose);

        LoadingStationBackwardPath.add(LoadingStagingPose);
        LoadingStationBackwardPath.add(ChargeStationBackupPose);

        Trajectory BlueTwoPieceWallForward =        TrajectoryGenerator.generateTrajectory(TwoPieceWallForwardPath, forwardConfig);
        Trajectory BlueTwoPieceCenterWallForward =  TrajectoryGenerator.generateTrajectory(TwoPieceCenterWallForwardPath, forwardConfig);
        Trajectory BlueTwoPieceCenterLoadForward =  TrajectoryGenerator.generateTrajectory(TwoPieceCenterLoadForwardPath, forwardConfig);
        Trajectory BlueTwoPieceLoadForward =        TrajectoryGenerator.generateTrajectory(TwoPieceLoadForwardPath, forwardConfig);

        Trajectory BlueTwoPieceWallBackward =        TrajectoryGenerator.generateTrajectory(TwoPieceWallBackwardPath, backwardConfig);
        Trajectory BlueTwoPieceCenterWallBackward =  TrajectoryGenerator.generateTrajectory(TwoPieceCenterWallBackwardPath, backwardConfig);
        Trajectory BlueTwoPieceCenterLoadBackward =  TrajectoryGenerator.generateTrajectory(TwoPieceCenterLoadBackwardPath, forwardConfig);
        Trajectory BlueTwoPieceLoadBackward =        TrajectoryGenerator.generateTrajectory(TwoPieceLoadBackwardPath, backwardConfig);

        // Generic
        
        Trajectory BlueWallScoringBackward =        TrajectoryGenerator.generateTrajectory(WallScoringBackwardPath, backwardConfig);
        Trajectory BlueCenterWallScoringBackward =  TrajectoryGenerator.generateTrajectory(CenterWallScoringBackwardPath, backwardConfig);
        Trajectory BlueCenterLoadScoringBackward =  TrajectoryGenerator.generateTrajectory(CenterLoadScoringBackwardPath, backwardConfig);
        Trajectory BlueLoadingScoringBackward =     TrajectoryGenerator.generateTrajectory(LoadingScoringBackwardPath, backwardConfig);

        Trajectory BlueWallPickupForward =          TrajectoryGenerator.generateTrajectory(WallPickupForwardPath, forwardConfig);
        Trajectory BlueCenterWallPickupForward =    TrajectoryGenerator.generateTrajectory(CenterWallPickupForwardPath, forwardConfig);
        Trajectory BlueCenterLoadPickupForward =    TrajectoryGenerator.generateTrajectory(CenterLoadPickupForwardPath, forwardConfig);
        Trajectory BlueLoadingPickupForward =       TrajectoryGenerator.generateTrajectory(LoadingPickupForwardPath, forwardConfig);

        // One Piece Skip

        Trajectory BlueWallSkipBackward =           TrajectoryGenerator.generateTrajectory(WallSkipBackwardPath, backwardConfig);
        Trajectory BlueCenterWallSkipBackward =     TrajectoryGenerator.generateTrajectory(CenterWallSkipBackwardPath, backwardConfig);
        Trajectory BlueCenterLoadSkipBackward =     TrajectoryGenerator.generateTrajectory(CenterLoadSkipBackwardPath, backwardConfig);
        Trajectory BlueLoadingSkipBackward =        TrajectoryGenerator.generateTrajectory(LoadingSkipBackwardPath, backwardConfig);
        
        // Two Piece

        Trajectory BlueWallPickupBackward =         TrajectoryGenerator.generateTrajectory(WallPickupBackwardPath, backwardConfig);
        Trajectory BlueCenterWallPickupBackward =   TrajectoryGenerator.generateTrajectory(CenterWallPickupBackwardPath, backwardConfig);
        Trajectory BlueCenterLoadPickupBackward =   TrajectoryGenerator.generateTrajectory(CenterLoadPickupBackwardPath, backwardConfig);
        Trajectory BlueLoadingPickupBackward =      TrajectoryGenerator.generateTrajectory(LoadingPickupBackwardPath, backwardConfig);

        Trajectory BlueWallScoringForward =         TrajectoryGenerator.generateTrajectory(WallScoringForwardPath, forwardConfig);
        Trajectory BlueCenterWallScoringForward =   TrajectoryGenerator.generateTrajectory(CenterWallScoringForwardPath, forwardConfig);
        Trajectory BlueCenterLoadScoringForward =   TrajectoryGenerator.generateTrajectory(CenterLoadScoringForwardPath, forwardConfig);
        Trajectory BlueLoadingScoringForward =      TrajectoryGenerator.generateTrajectory(LoadingScoringForwardPath, forwardConfig);

        // One Piece Balance

        Trajectory BlueWallStationBackward =        TrajectoryGenerator.generateTrajectory(WallStationBackwardPath, backwardConfig);
        Trajectory BlueCenterWallStationBackward =  TrajectoryGenerator.generateTrajectory(CenterWallStationBackwardPath, backwardConfig);
        Trajectory BlueCenterLoadStationBackward =  TrajectoryGenerator.generateTrajectory(CenterLoadStationBackwardPath, backwardConfig);
        Trajectory BlueLoadingStationBackward =     TrajectoryGenerator.generateTrajectory(LoadingStationBackwardPath, backwardConfig);

        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        CenterScoringPose = AllianceFlipUtil.apply(CenterScoringPose, Alliance.Red);
        LoadingScoringPose = AllianceFlipUtil.apply(LoadingScoringPose, Alliance.Red);
        CenterWallUTurnPose = AllianceFlipUtil.apply(CenterWallUTurnPose, Alliance.Red);
        CenterLoadUTurnPose = AllianceFlipUtil.apply(CenterLoadUTurnPose, Alliance.Red);
        WallBackupPose = AllianceFlipUtil.apply(WallBackupPose, Alliance.Red);
        CenterBackupPose = AllianceFlipUtil.apply(CenterBackupPose, Alliance.Red);
        LoadingBackupPose = AllianceFlipUtil.apply(LoadingBackupPose, Alliance.Red);
        WallPointTurnPose = AllianceFlipUtil.apply(WallPointTurnPose, Alliance.Red);
        CenterWallPointTurnPose = AllianceFlipUtil.apply(CenterWallPointTurnPose, Alliance.Red);
        CenterLoadPointTurnPose = AllianceFlipUtil.apply(CenterLoadPointTurnPose, Alliance.Red);
        LoadingPointTurnPose = AllianceFlipUtil.apply(LoadingPointTurnPose, Alliance.Red);
        WallStagingOffsetPose = AllianceFlipUtil.apply(WallStagingOffsetPose, Alliance.Red);
        CenterWallStagingOffsetPose = AllianceFlipUtil.apply(CenterWallStagingOffsetPose, Alliance.Red);
        CenterLoadStagingOffsetPose = AllianceFlipUtil.apply(CenterLoadStagingOffsetPose, Alliance.Red);
        LoadingStagingOffsetPose = AllianceFlipUtil.apply(LoadingStagingOffsetPose, Alliance.Red);
        WallStagingPose = AllianceFlipUtil.apply(WallStagingPose, Alliance.Red);
        CenterWallStagingPose = AllianceFlipUtil.apply(CenterWallStagingPose, Alliance.Red);
        CenterLoadStagingPose = AllianceFlipUtil.apply(CenterLoadStagingPose, Alliance.Red);
        LoadingStagingPose = AllianceFlipUtil.apply(LoadingStagingPose, Alliance.Red);
        ChargeStationBackupPose = AllianceFlipUtil.apply(ChargeStationBackupPose, Alliance.Red);
        TwoPieceWallScoringPose = AllianceFlipUtil.apply(TwoPieceWallScoringPose, Alliance.Red);
        TwoPieceCenterScoringPose = AllianceFlipUtil.apply(TwoPieceCenterScoringPose, Alliance.Red);
        TwoPieceLoadScoringPose = AllianceFlipUtil.apply(TwoPieceLoadScoringPose, Alliance.Red);

        TwoPieceWallForwardPath.clear();
        TwoPieceCenterWallForwardPath.clear();
        TwoPieceCenterLoadForwardPath.clear();
        TwoPieceLoadForwardPath.clear();

        TwoPieceWallForwardPath.add(TwoPieceWallScoringPose);
        TwoPieceWallForwardPath.add(WallStagingPose);

        TwoPieceCenterWallForwardPath.add(TwoPieceCenterScoringPose);
        TwoPieceCenterWallForwardPath.add(CenterWallStagingPose);
        
        TwoPieceCenterLoadForwardPath.add(TwoPieceCenterScoringPose);
        TwoPieceCenterLoadForwardPath.add(CenterLoadStagingPose);
        
        TwoPieceLoadForwardPath.add(TwoPieceLoadScoringPose);
        TwoPieceLoadForwardPath.add(LoadingStagingPose);

        TwoPieceWallBackwardPath.clear();
        TwoPieceCenterWallBackwardPath.clear();
        TwoPieceCenterLoadBackwardPath.clear();
        TwoPieceLoadBackwardPath.clear();

        TwoPieceWallBackwardPath.add(WallStagingPose);
        TwoPieceWallBackwardPath.add(TwoPieceWallScoringPose);

        TwoPieceCenterWallBackwardPath.add(CenterWallStagingPose);
        TwoPieceCenterWallBackwardPath.add(TwoPieceCenterScoringPose);
        
        TwoPieceCenterLoadBackwardPath.add(CenterLoadStagingPose);
        TwoPieceCenterLoadBackwardPath.add(TwoPieceCenterScoringPose);
        
        TwoPieceLoadBackwardPath.add(LoadingStagingPose);
        TwoPieceLoadBackwardPath.add(TwoPieceLoadScoringPose);

        WallScoringBackwardPath.clear();
        CenterWallScoringBackwardPath.clear();
        CenterLoadScoringBackwardPath.clear();
        LoadingScoringBackwardPath.clear();

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
        
        WallSkipBackwardPath.clear();
        CenterWallSkipBackwardPath.clear();
        CenterLoadSkipBackwardPath.clear();
        LoadingSkipBackwardPath.clear();

        WallSkipBackwardPath.add(WallScoringPose);
        WallSkipBackwardPath.add(WallBackupPose);
        WallSkipBackwardPath.add(ChargeStationBackupPose);

        CenterWallSkipBackwardPath.add(CenterScoringPose);
        CenterWallSkipBackwardPath.add(CenterBackupPose);
        CenterWallSkipBackwardPath.add(CenterWallUTurnPose);
        CenterWallSkipBackwardPath.add(ChargeStationBackupPose);
        
        CenterLoadSkipBackwardPath.add(CenterScoringPose);
        CenterLoadSkipBackwardPath.add(CenterBackupPose);
        CenterLoadSkipBackwardPath.add(CenterLoadUTurnPose);
        CenterLoadSkipBackwardPath.add(ChargeStationBackupPose);

        LoadingSkipBackwardPath.add(LoadingScoringPose);
        LoadingSkipBackwardPath.add(LoadingBackupPose);
        LoadingSkipBackwardPath.add(ChargeStationBackupPose);

        WallPickupForwardPath.clear();
        CenterWallPickupForwardPath.clear();
        CenterLoadPickupForwardPath.clear();
        LoadingPickupForwardPath.clear();

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

        WallPickupBackwardPath.clear();
        CenterWallPickupBackwardPath.clear();
        CenterLoadPickupBackwardPath.clear();
        LoadingPickupBackwardPath.clear();

        WallPickupBackwardPath.add(WallStagingPose);
        WallPickupBackwardPath.add(WallStagingOffsetPose);
        WallPickupBackwardPath.add(WallPointTurnPose);

        CenterWallPickupBackwardPath.add(CenterWallStagingPose);
        CenterWallPickupBackwardPath.add(CenterWallStagingOffsetPose);
        CenterWallPickupBackwardPath.add(CenterWallPointTurnPose);

        CenterLoadPickupBackwardPath.add(CenterLoadStagingPose);
        CenterLoadPickupBackwardPath.add(CenterLoadStagingOffsetPose);
        CenterLoadPickupBackwardPath.add(CenterLoadPointTurnPose);

        LoadingPickupBackwardPath.add(LoadingStagingPose);
        LoadingPickupBackwardPath.add(LoadingStagingOffsetPose);
        LoadingPickupBackwardPath.add(LoadingPointTurnPose);

        WallScoringForwardPath.clear();
        CenterWallScoringForwardPath.clear();
        CenterLoadScoringForwardPath.clear();
        LoadingScoringForwardPath.clear();

        WallScoringForwardPath.add(WallPointTurnPose);
        WallScoringForwardPath.add(WallBackupPose);
        WallScoringForwardPath.add(WallScoringPose);

        CenterWallScoringForwardPath.add(CenterWallPointTurnPose);
        CenterWallScoringForwardPath.add(CenterBackupPose);
        CenterWallScoringForwardPath.add(CenterScoringPose);

        CenterLoadScoringForwardPath.add(CenterLoadPointTurnPose);
        CenterLoadScoringForwardPath.add(CenterBackupPose);
        CenterLoadScoringForwardPath.add(CenterScoringPose);

        LoadingScoringForwardPath.add(LoadingPointTurnPose);
        LoadingScoringForwardPath.add(LoadingBackupPose);
        LoadingScoringForwardPath.add(LoadingScoringPose);

        WallStationBackwardPath.clear();
        CenterWallStationBackwardPath.clear();
        CenterLoadStationBackwardPath.clear();
        LoadingStationBackwardPath.clear();

        WallStationBackwardPath.add(WallStagingPose);
        WallStationBackwardPath.add(ChargeStationBackupPose);

        CenterWallStationBackwardPath.add(CenterWallStagingPose);
        CenterWallStationBackwardPath.add(ChargeStationBackupPose);

        CenterLoadStationBackwardPath.add(CenterLoadStagingPose);
        CenterLoadStationBackwardPath.add(ChargeStationBackupPose);

        LoadingStationBackwardPath.add(LoadingStagingPose);
        LoadingStationBackwardPath.add(ChargeStationBackupPose);

        Trajectory RedTwoPieceWallForward =        TrajectoryGenerator.generateTrajectory(TwoPieceWallForwardPath, forwardConfig);
        Trajectory RedTwoPieceCenterWallForward =  TrajectoryGenerator.generateTrajectory(TwoPieceCenterWallForwardPath, forwardConfig);
        Trajectory RedTwoPieceCenterLoadForward =  TrajectoryGenerator.generateTrajectory(TwoPieceCenterLoadForwardPath, forwardConfig);
        Trajectory RedTwoPieceLoadForward =        TrajectoryGenerator.generateTrajectory(TwoPieceLoadForwardPath, forwardConfig);

        Trajectory RedTwoPieceWallBackward =        TrajectoryGenerator.generateTrajectory(TwoPieceWallBackwardPath, backwardConfig);
        Trajectory RedTwoPieceCenterWallBackward =  TrajectoryGenerator.generateTrajectory(TwoPieceCenterWallBackwardPath, backwardConfig);
        Trajectory RedTwoPieceCenterLoadBackward =  TrajectoryGenerator.generateTrajectory(TwoPieceCenterLoadBackwardPath, forwardConfig);
        Trajectory RedTwoPieceLoadBackward =        TrajectoryGenerator.generateTrajectory(TwoPieceLoadBackwardPath, backwardConfig);

        // Generic
        
        Trajectory RedWallScoringBackward =       TrajectoryGenerator.generateTrajectory(WallScoringBackwardPath, backwardConfig);
        Trajectory RedCenterWallScoringBackward = TrajectoryGenerator.generateTrajectory(CenterWallScoringBackwardPath, backwardConfig);
        Trajectory RedCenterLoadScoringBackward = TrajectoryGenerator.generateTrajectory(CenterLoadScoringBackwardPath, backwardConfig);
        Trajectory RedLoadingScoringBackward =    TrajectoryGenerator.generateTrajectory(LoadingScoringBackwardPath, backwardConfig);

        Trajectory RedWallPickupForward =         TrajectoryGenerator.generateTrajectory(WallPickupForwardPath, forwardConfig);
        Trajectory RedCenterWallPickupForward =   TrajectoryGenerator.generateTrajectory(CenterWallPickupForwardPath, forwardConfig);
        Trajectory RedCenterLoadPickupForward =   TrajectoryGenerator.generateTrajectory(CenterLoadPickupForwardPath, forwardConfig);
        Trajectory RedLoadingPickupForward =      TrajectoryGenerator.generateTrajectory(LoadingPickupForwardPath, forwardConfig);

        // One Piece Skip

        Trajectory RedWallSkipBackward =           TrajectoryGenerator.generateTrajectory(WallSkipBackwardPath, backwardConfig);
        Trajectory RedCenterWallSkipBackward =     TrajectoryGenerator.generateTrajectory(CenterWallSkipBackwardPath, backwardConfig);
        Trajectory RedCenterLoadSkipBackward =     TrajectoryGenerator.generateTrajectory(CenterLoadSkipBackwardPath, backwardConfig);
        Trajectory RedLoadingSkipBackward =        TrajectoryGenerator.generateTrajectory(LoadingSkipBackwardPath, backwardConfig);

        // Two Piece

        Trajectory RedWallPickupBackward =        TrajectoryGenerator.generateTrajectory(WallPickupBackwardPath, backwardConfig);
        Trajectory RedCenterWallPickupBackward =  TrajectoryGenerator.generateTrajectory(CenterWallPickupBackwardPath, backwardConfig);
        Trajectory RedCenterLoadPickupBackward =  TrajectoryGenerator.generateTrajectory(CenterLoadPickupBackwardPath, backwardConfig);
        Trajectory RedLoadingPickupBackward =     TrajectoryGenerator.generateTrajectory(LoadingPickupBackwardPath, backwardConfig);

        Trajectory RedWallScoringForward =        TrajectoryGenerator.generateTrajectory(WallScoringForwardPath, forwardConfig);
        Trajectory RedCenterWallScoringForward =  TrajectoryGenerator.generateTrajectory(CenterWallScoringForwardPath, forwardConfig);
        Trajectory RedCenterLoadScoringForward =  TrajectoryGenerator.generateTrajectory(CenterLoadScoringForwardPath, forwardConfig);
        Trajectory RedLoadingScoringForward =     TrajectoryGenerator.generateTrajectory(LoadingScoringForwardPath, forwardConfig);

        // One Piece Balance

        Trajectory RedWallStationBackward =       TrajectoryGenerator.generateTrajectory(WallStationBackwardPath, backwardConfig);
        Trajectory RedCenterWallStationBackward = TrajectoryGenerator.generateTrajectory(CenterWallStationBackwardPath, backwardConfig);
        Trajectory RedCenterLoadStationBackward = TrajectoryGenerator.generateTrajectory(CenterLoadStationBackwardPath, backwardConfig);
        Trajectory RedLoadingStationBackward =    TrajectoryGenerator.generateTrajectory(LoadingStationBackwardPath, backwardConfig);

        TwoPieceForward = new Trajectory[2][4];
        for(AutoConfiguration.StartPosition startPosition : AutoConfiguration.StartPosition.values()) {
            for(Alliance alliance : Alliance.values()) {
                if(alliance != Alliance.Invalid) {
                    Trajectory traj = null;
                    switch(startPosition) {
                        case Wall:          traj = (alliance == Alliance.Red ? RedTwoPieceWallForward : BlueTwoPieceWallForward);               break;
                        case CenterWall:    traj = (alliance == Alliance.Red ? RedTwoPieceCenterWallForward : BlueTwoPieceCenterWallForward);   break;
                        case CenterLoad:    traj = (alliance == Alliance.Red ? RedTwoPieceCenterLoadForward : BlueTwoPieceCenterLoadForward);   break;
                        case Loading:       traj = (alliance == Alliance.Red ? RedTwoPieceLoadForward : BlueTwoPieceLoadForward);         break;
                    }
                    TwoPieceForward[alliance.ordinal()][startPosition.ordinal()] = traj;
                }
            }
        }

        TwoPieceBackward = new Trajectory[2][4];
        for(AutoConfiguration.StartPosition startPosition : AutoConfiguration.StartPosition.values()) {
            for(Alliance alliance : Alliance.values()) {
                if(alliance != Alliance.Invalid) {
                    Trajectory traj = null;
                    switch(startPosition) {
                        case Wall:          traj = (alliance == Alliance.Red ? RedTwoPieceWallBackward : BlueTwoPieceWallBackward);               break;
                        case CenterWall:    traj = (alliance == Alliance.Red ? RedTwoPieceCenterWallBackward : BlueTwoPieceCenterWallBackward);   break;
                        case CenterLoad:    traj = (alliance == Alliance.Red ? RedTwoPieceCenterLoadBackward : BlueTwoPieceCenterLoadBackward);   break;
                        case Loading:       traj = (alliance == Alliance.Red ? RedTwoPieceLoadBackward : BlueTwoPieceLoadBackward);         break;
                    }
                    TwoPieceBackward[alliance.ordinal()][startPosition.ordinal()] = traj;
                }
            }
        }

        ScoringBackward = new Trajectory[2][4];
        for(AutoConfiguration.StartPosition startPosition : AutoConfiguration.StartPosition.values()) {
            for(Alliance alliance : Alliance.values()) {
                if(alliance != Alliance.Invalid) {
                    Trajectory traj = null;
                    switch(startPosition) {
                        case Wall:          traj = (alliance == Alliance.Red ? RedWallScoringBackward : BlueWallScoringBackward);               break;
                        case CenterWall:    traj = (alliance == Alliance.Red ? RedCenterWallScoringBackward : BlueCenterWallScoringBackward);   break;
                        case CenterLoad:    traj = (alliance == Alliance.Red ? RedCenterLoadScoringBackward : BlueCenterLoadScoringBackward);   break;
                        case Loading:       traj = (alliance == Alliance.Red ? RedLoadingScoringBackward : BlueLoadingScoringBackward);         break;
                    }
                    ScoringBackward[alliance.ordinal()][startPosition.ordinal()] = traj;
                }
            }
        }

        PickupForward = new Trajectory[2][4];
        for(AutoConfiguration.StartPosition startPosition : AutoConfiguration.StartPosition.values()) {
            for(Alliance alliance : Alliance.values()) {
                if(alliance != Alliance.Invalid) {
                    Trajectory traj = null;
                    switch(startPosition) {
                        case Wall:          traj = (alliance == Alliance.Red ? RedWallPickupForward : BlueWallPickupForward);               break;
                        case CenterWall:    traj = (alliance == Alliance.Red ? RedCenterWallPickupForward : BlueCenterWallPickupForward);   break;
                        case CenterLoad:    traj = (alliance == Alliance.Red ? RedCenterLoadPickupForward : BlueCenterLoadPickupForward);   break;
                        case Loading:       traj = (alliance == Alliance.Red ? RedLoadingPickupForward : BlueLoadingPickupForward);         break;
                    }
                    PickupForward[alliance.ordinal()][startPosition.ordinal()] = traj;
                }
            }
        }

        PickupBackward = new Trajectory[2][4];
        for(AutoConfiguration.StartPosition startPosition : AutoConfiguration.StartPosition.values()) {
            for(Alliance alliance : Alliance.values()) {
                if(alliance != Alliance.Invalid) {
                    Trajectory traj = null;
                    switch(startPosition) {
                        case Wall:          traj = (alliance == Alliance.Red ? RedWallPickupBackward : BlueWallPickupBackward);               break;
                        case CenterWall:    traj = (alliance == Alliance.Red ? RedCenterWallPickupBackward : BlueCenterWallPickupBackward);   break;
                        case CenterLoad:    traj = (alliance == Alliance.Red ? RedCenterLoadPickupBackward : BlueCenterLoadPickupBackward);   break;
                        case Loading:       traj = (alliance == Alliance.Red ? RedLoadingPickupBackward : BlueLoadingPickupBackward);         break;
                    }
                    PickupBackward[alliance.ordinal()][startPosition.ordinal()] = traj;
                }
            }
        }

        ScoringForward = new Trajectory[2][4];
        for(AutoConfiguration.StartPosition startPosition : AutoConfiguration.StartPosition.values()) {
            for(Alliance alliance : Alliance.values()) {
                if(alliance != Alliance.Invalid) {
                    Trajectory traj = null;
                    switch(startPosition) {
                        case Wall:          traj = (alliance == Alliance.Red ? RedWallScoringForward : BlueWallScoringForward);               break;
                        case CenterWall:    traj = (alliance == Alliance.Red ? RedCenterWallScoringForward : BlueCenterWallScoringForward);   break;
                        case CenterLoad:    traj = (alliance == Alliance.Red ? RedCenterLoadScoringForward : BlueCenterLoadScoringForward);   break;
                        case Loading:       traj = (alliance == Alliance.Red ? RedLoadingScoringForward : BlueLoadingScoringForward);         break;
                    }
                    ScoringForward[alliance.ordinal()][startPosition.ordinal()] = traj;
                }
            }
        }
        
        StationBackward = new Trajectory[2][4];
        for(AutoConfiguration.StartPosition startPosition : AutoConfiguration.StartPosition.values()) {
            for(Alliance alliance : Alliance.values()) {
                if(alliance != Alliance.Invalid) {
                    Trajectory traj = null;
                    switch(startPosition) {
                        case Wall:          traj = (alliance == Alliance.Red ? RedWallStationBackward : BlueWallStationBackward);               break;
                        case CenterWall:    traj = (alliance == Alliance.Red ? RedCenterWallStationBackward : BlueCenterWallStationBackward);   break;
                        case CenterLoad:    traj = (alliance == Alliance.Red ? RedCenterLoadStationBackward : BlueCenterLoadStationBackward);   break;
                        case Loading:       traj = (alliance == Alliance.Red ? RedLoadingStationBackward : BlueLoadingStationBackward);         break;
                    }
                    StationBackward[alliance.ordinal()][startPosition.ordinal()] = traj;
                }
            }
        }

        SkipBackward = new Trajectory[2][4];
        for(AutoConfiguration.StartPosition startPosition : AutoConfiguration.StartPosition.values()) {
            for(Alliance alliance : Alliance.values()) {
                if(alliance != Alliance.Invalid) {
                    Trajectory traj = null;
                    switch(startPosition) {
                        case Wall:          traj = (alliance == Alliance.Red ? RedWallSkipBackward : BlueWallSkipBackward);               break;
                        case CenterWall:    traj = (alliance == Alliance.Red ? RedCenterWallSkipBackward : BlueCenterWallSkipBackward);   break;
                        case CenterLoad:    traj = (alliance == Alliance.Red ? RedCenterLoadSkipBackward : BlueCenterLoadSkipBackward);   break;
                        case Loading:       traj = (alliance == Alliance.Red ? RedLoadingSkipBackward : BlueLoadingSkipBackward);         break;
                    }
                    SkipBackward[alliance.ordinal()][startPosition.ordinal()] = traj;
                }
            }
        }
    }

    public static void loadTrajectories() {
        System.out.println("Loaded trajectories");
    }
}
