package frc.robot.auto.modes;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfiguration;
import frc.robot.auto.AutoConfiguration;
import frc.robot.auto.AutoTrajectories;
import frc.robot.auto.AutoConfiguration.GamePiece;
import frc.robot.auto.AutoConfiguration.StartPosition;
import frc.robot.auto.actions.ArmCommandAction;
import frc.robot.auto.actions.IntakeCommandAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.RamseteFollowerAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.WaitUntilAction;
import frc.robot.subsystems.arm.ArmCommand;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.arm.ArmStatus.NodeEnum;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class TwoPieceAuto extends AutoMode {
    private final ArmStatus armStatus = ArmStatus.getInstance();

    public TwoPieceAuto(AutoConfiguration config) {
        Trajectory[] trajectories = new Trajectory[4];

        trajectories[0] = AutoTrajectories.ScoringBackward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];
        trajectories[1] = AutoTrajectories.PickupForward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];
        trajectories[2] = AutoTrajectories.PickupBackward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];
        trajectories[3] = AutoTrajectories.ScoringForward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];

        startConfiguration = new RobotConfiguration(trajectories[0].getInitialPose(), ArmPose.Preset.AUTO_START, ArmState.Hold);

        RamseteController ramseteController = new RamseteController(2, 0.7);

        addAction(new ArmCommandAction(new ArmCommand(ArmState.Extend).setTargetNode(config.startingPiece == GamePiece.Cube ? NodeEnum.TopCenter : (config.startingPosition == StartPosition.Loading ? NodeEnum.TopWall : NodeEnum.TopLoading))));
        addAction(new WaitUntilAction() {
            @Override
            public boolean Condition() {
                return armStatus.getArmState() == ArmState.Adjust;
            }
        });
        addAction(new ArmCommandAction(new ArmCommand(ArmState.Release)));
        addAction(new WaitUntilAction() {
            @Override
            public boolean Condition() {
                return armStatus.getCurrentArmPose() == ArmPose.Preset.DEFENSE;
            }
        });
        addAction(new RamseteFollowerAction(trajectories[0], ramseteController));
        addAction(new ParallelAction(
            new SeriesAction(
                new IntakeCommandAction(new IntakeCommand(IntakeState.Grab)),
                new WaitUntilAction() {
                    @Override
                    public boolean Condition() {
                        return armStatus.getArmState() == ArmState.Hold;
                    };
                },
                new ArmCommandAction(new ArmCommand(ArmState.Align))
            ),
            new SeriesAction(
                new RamseteFollowerAction(trajectories[1], ramseteController),
                new RamseteFollowerAction(trajectories[2], ramseteController),
                new RamseteFollowerAction(trajectories[3], ramseteController)
            )
        ));
        NodeEnum secondNode = NodeEnum.TopCenter;
        if(config.stagedPieces[config.startingPosition.ordinal()] == GamePiece.Cone) { // If second piece is a cone, override cube node
            if (config.startingPosition == StartPosition.Loading ^ config.startingPiece == config.stagedPieces[config.startingPosition.ordinal()]) { // Choose closest to center cone node, unless already placed a cone
                secondNode = NodeEnum.TopWall;
            } else {
                secondNode = NodeEnum.TopLoading;
            }
        }
        addAction(new ArmCommandAction(new ArmCommand(ArmState.Extend).setTargetNode(secondNode)));
        addAction(new WaitUntilAction() {
            @Override
            public boolean Condition() {
                return armStatus.getArmState() == ArmState.Adjust;
            }
        });
        addAction(new ArmCommandAction(new ArmCommand(ArmState.Release)));
    }
}
