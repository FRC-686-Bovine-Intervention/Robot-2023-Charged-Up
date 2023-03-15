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
import frc.robot.RobotConfiguration;
import frc.robot.auto.actions.RamseteFollowerAction;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.util.AllianceFlipUtil;

public class WallSideTwoPieceAuto extends AutoMode {
    static {
        double kMaxVelocityMPS =      Units.inchesToMeters(48);
        double kMaxAccelerationMPSS = kMaxVelocityMPS * 2;

        TrajectoryConfig forwardConfig = new TrajectoryConfig(kMaxVelocityMPS, kMaxAccelerationMPSS);
        TrajectoryConfig backwardConfig = new TrajectoryConfig(kMaxVelocityMPS, kMaxAccelerationMPSS);
        backwardConfig.setReversed(true);
        
        double onePointTurnRadius = 24;

        Pose2d scoringPose = new Pose2d(
            new Translation2d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0)
            ),
            Rotation2d.fromDegrees(180)
        ); //TODO Initial auto pose

        Pose2d backupChargeStation = new Pose2d(
            new Translation2d(
                FieldDimensions.StagingLocations.translations[0].getX() - Units.inchesToMeters(2 * onePointTurnRadius + Constants.kCenterToIntake),
                scoringPose.getTranslation().getY()
            ),
            Rotation2d.fromDegrees(180)
        );
        Pose2d onePointTurn = new Pose2d(
            FieldDimensions.StagingLocations.translations[0].plus(new Translation2d(
                Units.inchesToMeters(-(onePointTurnRadius + Constants.kCenterToIntake)),
                Units.inchesToMeters(onePointTurnRadius)
            )),
            Rotation2d.fromDegrees(270)
        );
        Pose2d stagingPoint = new Pose2d(
            FieldDimensions.StagingLocations.translations[0].plus(new Translation2d(
                Units.inchesToMeters(-Constants.kCenterToIntake),
                Units.inchesToMeters(0)
            )), 
            Rotation2d.fromDegrees(0)
        );

        scoringPose =           AllianceFlipUtil.apply(scoringPose);
        backupChargeStation =   AllianceFlipUtil.apply(backupChargeStation);
        onePointTurn =          AllianceFlipUtil.apply(onePointTurn);
        stagingPoint =          AllianceFlipUtil.apply(stagingPoint);

        ArrayList<Pose2d> backwardFromScoringPath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> forwardToStagePath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> backwardFromStagePath = new ArrayList<Pose2d>();
        ArrayList<Pose2d> forwardToScoringPath = new ArrayList<Pose2d>();

        backwardFromScoringPath.add(scoringPose);
        backwardFromScoringPath.add(backupChargeStation);
        backwardFromScoringPath.add(onePointTurn);

        forwardToStagePath.add(onePointTurn);
        forwardToStagePath.add(stagingPoint);

        backwardFromStagePath.add(stagingPoint);
        backwardFromStagePath.add(onePointTurn);

        forwardToScoringPath.add(onePointTurn);
        forwardToScoringPath.add(backupChargeStation);
        forwardToScoringPath.add(scoringPose);
        
        traj1 = TrajectoryGenerator.generateTrajectory(backwardFromScoringPath, backwardConfig);
        traj2 = TrajectoryGenerator.generateTrajectory(forwardToStagePath, forwardConfig);
        traj3 = TrajectoryGenerator.generateTrajectory(backwardFromStagePath, backwardConfig);
        traj4 = TrajectoryGenerator.generateTrajectory(forwardToScoringPath, forwardConfig);
    }

    private static final Trajectory traj1;
    private static final Trajectory traj2;
    private static final Trajectory traj3;
    private static final Trajectory traj4;

    public WallSideTwoPieceAuto() {
        startConfiguration = new RobotConfiguration(traj1.getInitialPose(), ArmPose.Preset.AUTO_START, ArmState.Hold);

        RamseteController ramseteController = new RamseteController(2, 0.7);

        addAction(new RamseteFollowerAction(traj1, ramseteController));
        addAction(new RamseteFollowerAction(traj2, ramseteController));
        addAction(new RamseteFollowerAction(traj3, ramseteController));
        addAction(new RamseteFollowerAction(traj4, ramseteController));
    }
}
