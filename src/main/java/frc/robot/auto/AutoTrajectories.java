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
    public static final Trajectory[][] ScoringBackward;
    public static final Trajectory[][] PickupForward;

    // Two Piece
    public static final Trajectory[][] PickupBackward;
    public static final Trajectory[][] ScoringForward;

    // One Piece Balance
    public static final Trajectory[][] StationBackward;

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
        
        Trajectory BlueWallScoringBackward =       TrajectoryGenerator.generateTrajectory(WallScoringBackwardPath, backwardConfig);
        Trajectory BlueCenterWallScoringBackward = TrajectoryGenerator.generateTrajectory(CenterWallScoringBackwardPath, backwardConfig);
        Trajectory BlueCenterLoadScoringBackward = TrajectoryGenerator.generateTrajectory(CenterLoadScoringBackwardPath, backwardConfig);
        Trajectory BlueLoadingScoringBackward =    TrajectoryGenerator.generateTrajectory(LoadingScoringBackwardPath, backwardConfig);

        Trajectory BlueWallPickupForward =         TrajectoryGenerator.generateTrajectory(WallPickupForwardPath, forwardConfig);
        Trajectory BlueCenterWallPickupForward =   TrajectoryGenerator.generateTrajectory(CenterWallPickupForwardPath, forwardConfig);
        Trajectory BlueCenterLoadPickupForward =   TrajectoryGenerator.generateTrajectory(CenterLoadPickupForwardPath, forwardConfig);
        Trajectory BlueLoadingPickupForward =      TrajectoryGenerator.generateTrajectory(LoadingPickupForwardPath, forwardConfig);

        // Two Piece

        Trajectory BlueWallPickupBackward =        TrajectoryGenerator.generateTrajectory(WallPickupBackwardPath, backwardConfig);
        Trajectory BlueCenterWallPickupBackward =  TrajectoryGenerator.generateTrajectory(CenterWallPickupBackwardPath, backwardConfig);
        Trajectory BlueCenterLoadPickupBackward =  TrajectoryGenerator.generateTrajectory(CenterLoadPickupBackwardPath, backwardConfig);
        Trajectory BlueLoadingPickupBackward =     TrajectoryGenerator.generateTrajectory(LoadingPickupBackwardPath, backwardConfig);

        Trajectory BlueWallScoringForward =        TrajectoryGenerator.generateTrajectory(WallScoringForwardPath, forwardConfig);
        Trajectory BlueCenterWallScoringForward =  TrajectoryGenerator.generateTrajectory(CenterWallScoringForwardPath, forwardConfig);
        Trajectory BlueCenterLoadScoringForward =  TrajectoryGenerator.generateTrajectory(CenterLoadScoringForwardPath, forwardConfig);
        Trajectory BlueLoadingScoringForward =     TrajectoryGenerator.generateTrajectory(LoadingScoringForwardPath, forwardConfig);

        // One Piece Balance

        Trajectory BlueWallStationBackward =       TrajectoryGenerator.generateTrajectory(WallStationBackwardPath, backwardConfig);
        Trajectory BlueCenterWallStationBackward = TrajectoryGenerator.generateTrajectory(CenterWallStationBackwardPath, backwardConfig);
        Trajectory BlueCenterLoadStationBackward = TrajectoryGenerator.generateTrajectory(CenterLoadStationBackwardPath, backwardConfig);
        Trajectory BlueLoadingStationBackward =    TrajectoryGenerator.generateTrajectory(LoadingStationBackwardPath, backwardConfig);

        WallScoringPose = AllianceFlipUtil.apply(WallScoringPose, Alliance.Red);
        CenterScoringPose = AllianceFlipUtil.apply(CenterScoringPose, Alliance.Red);
        LoadingScoringPose = AllianceFlipUtil.apply(LoadingScoringPose, Alliance.Red);
        WallBackupPose = AllianceFlipUtil.apply(WallBackupPose, Alliance.Red);
        CenterBackupPose = AllianceFlipUtil.apply(CenterBackupPose, Alliance.Red);
        LoadingBackupPose = AllianceFlipUtil.apply(LoadingBackupPose, Alliance.Red);
        WallPointTurnPose = AllianceFlipUtil.apply(WallPointTurnPose, Alliance.Red);
        CenterWallPointTurnPose = AllianceFlipUtil.apply(CenterWallPointTurnPose, Alliance.Red);
        CenterLoadPointTurnPose = AllianceFlipUtil.apply(CenterLoadPointTurnPose, Alliance.Red);
        LoadingPointTurnPose = AllianceFlipUtil.apply(LoadingPointTurnPose, Alliance.Red);
        WallStagingOffsetPose = AllianceFlipUtil.apply(WallStagingOffsetPose, Alliance.Red);
        CenterWallStagingOffsetPose = AllianceFlipUtil.apply(CenterWallStagingOffsetPose, Alliance.Red);
        LoadingStagingOffsetPose = AllianceFlipUtil.apply(LoadingStagingOffsetPose, Alliance.Red);
        WallStagingPose = AllianceFlipUtil.apply(WallStagingPose, Alliance.Red);
        CenterWallStagingPose = AllianceFlipUtil.apply(CenterWallStagingPose, Alliance.Red);
        CenterLoadStagingPose = AllianceFlipUtil.apply(CenterLoadStagingPose, Alliance.Red);
        LoadingStagingPose = AllianceFlipUtil.apply(LoadingStagingPose, Alliance.Red);
        ChargeStationBackupPose = AllianceFlipUtil.apply(ChargeStationBackupPose, Alliance.Red);

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

        WallScoringForwardPath.clear();
        CenterWallScoringForwardPath.clear();
        CenterLoadScoringForwardPath.clear();
        LoadingScoringForwardPath.clear();

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

        WallStationBackwardPath.clear();
        CenterWallStationBackwardPath.clear();
        CenterLoadStationBackwardPath.clear();
        LoadingStationBackwardPath.clear();

        WallPickupForwardPath.add(WallStagingPose);
        WallPickupForwardPath.add(ChargeStationBackupPose);

        CenterWallStationBackwardPath.add(CenterWallStagingPose);
        CenterWallStationBackwardPath.add(ChargeStationBackupPose);

        CenterLoadStationBackwardPath.add(CenterLoadStagingPose);
        CenterLoadStationBackwardPath.add(ChargeStationBackupPose);

        LoadingStationBackwardPath.add(LoadingStagingPose);
        LoadingStationBackwardPath.add(ChargeStationBackupPose);

        // Generic
        
        Trajectory RedWallScoringBackward =       TrajectoryGenerator.generateTrajectory(WallScoringBackwardPath, backwardConfig);
        Trajectory RedCenterWallScoringBackward = TrajectoryGenerator.generateTrajectory(CenterWallScoringBackwardPath, backwardConfig);
        Trajectory RedCenterLoadScoringBackward = TrajectoryGenerator.generateTrajectory(CenterLoadScoringBackwardPath, backwardConfig);
        Trajectory RedLoadingScoringBackward =    TrajectoryGenerator.generateTrajectory(LoadingScoringBackwardPath, backwardConfig);

        Trajectory RedWallPickupForward =         TrajectoryGenerator.generateTrajectory(WallPickupForwardPath, forwardConfig);
        Trajectory RedCenterWallPickupForward =   TrajectoryGenerator.generateTrajectory(CenterWallPickupForwardPath, forwardConfig);
        Trajectory RedCenterLoadPickupForward =   TrajectoryGenerator.generateTrajectory(CenterLoadPickupForwardPath, forwardConfig);
        Trajectory RedLoadingPickupForward =      TrajectoryGenerator.generateTrajectory(LoadingPickupForwardPath, forwardConfig);

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
    }
}
