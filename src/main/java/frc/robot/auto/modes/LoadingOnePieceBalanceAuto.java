package frc.robot.auto.modes;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldDimensions;
import frc.robot.auto.actions.DriveOnChargeStationEdgeAction;
import frc.robot.auto.actions.DriverAssistCommandAction;
import frc.robot.auto.actions.RamseteFollowerAction;
import frc.robot.subsystems.driverAssist.DriverAssistCommand;
import frc.robot.subsystems.driverAssist.DriverAssistStatus.DriverAssistState;
import frc.robot.util.AllianceFlipUtil;

public class LoadingOnePieceBalanceAuto extends AutoMode {
    static {
        double kMaxVelocityMPS =      Units.inchesToMeters(48);
        double kMaxAccelerationMPSS = kMaxVelocityMPS * 2;

        TrajectoryConfig forwardConfig = new TrajectoryConfig(kMaxVelocityMPS, kMaxAccelerationMPSS);
        TrajectoryConfig backwardConfig = new TrajectoryConfig(kMaxVelocityMPS, kMaxAccelerationMPSS);
        backwardConfig.setReversed(true);
        
        double onePointTurnRadius = 24;

        Pose2d scoringPose = new Pose2d(
            new Translation2d(
                FieldDimensions.Grids.outerX + Constants.kCenterToFrontBumper,
                FieldDimensions.Grids.nodeFirstY + FieldDimensions.Grids.nodeSeparationY * 7
            ),
            Rotation2d.fromDegrees(180)
        ); //TODO Initial auto pose

        Pose2d backupChargeStation = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.translations[3].getX() - Units.inchesToMeters(2 * onePointTurnRadius + Constants.kCenterToIntake),
                scoringPose.getTranslation().getY()
            ),
            Rotation2d.fromDegrees(180)
        );
        Pose2d onePointTurn = new Pose2d(
            FieldDimensions.StagingLocations.translations[3].plus(new Translation2d(
                Units.inchesToMeters(-(onePointTurnRadius + Constants.kCenterToIntake)),
                Units.inchesToMeters(onePointTurnRadius)
            )),
            Rotation2d.fromDegrees(270)
        );
        Pose2d stagingPoint = new Pose2d(
            FieldDimensions.StagingLocations.translations[3].plus(new Translation2d(
                Units.inchesToMeters(-Constants.kCenterToIntake),
                Units.inchesToMeters(0)
            )), 
            Rotation2d.fromDegrees(0)
        );
        Pose2d preChargeStationPoint = new Pose2d(
            new Translation2d(
                FieldDimensions.Community.chargingStationOuterX + Constants.kCenterToFrontBumper,
                FieldDimensions.Community.chargingStationRightY + FieldDimensions.Community.chargingStationWidth / 2
            ),
            Rotation2d.fromDegrees(0)
        );

        scoringPose =           AllianceFlipUtil.apply(scoringPose);
        backupChargeStation =   AllianceFlipUtil.apply(backupChargeStation);
        onePointTurn =          AllianceFlipUtil.apply(onePointTurn);
        stagingPoint =          AllianceFlipUtil.apply(stagingPoint);
        preChargeStationPoint = AllianceFlipUtil.apply(preChargeStationPoint);

        ArrayList<Pose2d> backwardFromScoringPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> forwardToStagePath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> backwardFromStagePath = new ArrayList<Pose2d>();

        backwardFromScoringPath.add(scoringPose);
        backwardFromScoringPath.add(backupChargeStation);
        backwardFromScoringPath.add(onePointTurn);

        forwardToStagePath.add(onePointTurn);
        forwardToStagePath.add(stagingPoint);

        backwardFromStagePath.add(stagingPoint);
        backwardFromStagePath.add(preChargeStationPoint);

        traj1 = TrajectoryGenerator.generateTrajectory(backwardFromScoringPath, backwardConfig);
        traj2 = TrajectoryGenerator.generateTrajectory(forwardToStagePath, forwardConfig);
        traj3 = TrajectoryGenerator.generateTrajectory(backwardFromStagePath, backwardConfig);
    }

    private static final Trajectory traj1;
    private static final Trajectory traj2;
    private static final Trajectory traj3;

    public LoadingOnePieceBalanceAuto() {
        initialPose = traj1.getInitialPose();

        RamseteController ramseteController = new RamseteController(2, 0.7);

        addAction(new RamseteFollowerAction(traj1, ramseteController));
        addAction(new RamseteFollowerAction(traj2, ramseteController));
        addAction(new RamseteFollowerAction(traj3, ramseteController));
        addAction(new DriveOnChargeStationEdgeAction(true));
        addAction(new DriverAssistCommandAction(new DriverAssistCommand(DriverAssistState.AutoBalance)));
    }
}
