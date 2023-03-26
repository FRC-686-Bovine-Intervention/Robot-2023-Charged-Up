package frc.robot.auto.modes;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfiguration;
import frc.robot.auto.actions.ArmCommandAction;
import frc.robot.auto.actions.ConditionalAction;
import frc.robot.auto.actions.DriveOnChargeStationEdgeAction;
import frc.robot.auto.actions.DrivePercentAction;
import frc.robot.auto.actions.DriverAssistCommandAction;
import frc.robot.auto.actions.IntakeCommandAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.RamseteFollowerAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.WaitUntilAction;
import frc.robot.auto.autoManager.AutoConfiguration;
import frc.robot.auto.autoManager.AutoConfiguration.GamePiece;
import frc.robot.auto.autoManager.AutoConfiguration.StartPosition;
import frc.robot.subsystems.arm.ArmCommand;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.arm.ArmStatus.NodeEnum;
import frc.robot.subsystems.driverAssist.DriverAssistCommand;
import frc.robot.subsystems.driverAssist.DriverAssistStatus.DriverAssistState;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeStatus;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class OneGrabBalanceAuto extends AutoMode {
    private final ArmStatus armStatus = ArmStatus.getInstance();
    private final IntakeStatus intakeStatus = IntakeStatus.getInstance();

    public OneGrabBalanceAuto(AutoConfiguration config) {
        
        Trajectory[] trajectories = new Trajectory[3];

        trajectories[0] = AutoTrajectories.ScoringBackward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];
        trajectories[1] = AutoTrajectories.PickupForward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];
        trajectories[2] = AutoTrajectories.StationBackward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];

        startConfiguration = new RobotConfiguration(trajectories[0].getInitialPose(), ArmPose.Preset.AUTO_START, ArmState.Hold);

        RamseteController ramseteController = new RamseteController(2, 0.7);

        addAction(new WaitUntilAction(() -> armStatus.getCurrentArmPose() == ArmPose.Preset.DEFENSE));
        addAction(new ArmCommandAction(new ArmCommand(ArmState.AlignNode).setTargetNode(config.startingPiece == GamePiece.Cube ? NodeEnum.TopCenter : (config.startingPosition == StartPosition.Loading ? NodeEnum.TopWall : NodeEnum.TopLoading))));
        addAction(new WaitUntilAction(() -> armStatus.getArmState() == ArmState.Adjust));
        addAction(new ArmCommandAction(new ArmCommand(ArmState.Release)));
        addAction(new WaitUntilAction(() -> armStatus.getTargetArmPose() == ArmPose.Preset.DEFENSE));
        addAction(new RamseteFollowerAction(trajectories[0], ramseteController));
        addAction(new IntakeCommandAction(new IntakeCommand(IntakeState.Grab)));
        addAction(new RamseteFollowerAction(trajectories[1], ramseteController));
        addAction(new ParallelAction(
            new SeriesAction(
                new RamseteFollowerAction(trajectories[2], ramseteController),
                new DriveOnChargeStationEdgeAction(true).setTimeout(3),
                new DrivePercentAction(12, -0.3),
                new DriverAssistCommandAction(new DriverAssistCommand(DriverAssistState.AutoBalance))
            ),
            new SeriesAction(
                new WaitUntilAction(() -> intakeStatus.getIntakeState() == IntakeState.Hold).setTimeout(0.5),
                new ConditionalAction(() -> intakeStatus.getIntakeState() == IntakeState.Hold, 
                    new WaitUntilAction(() -> armStatus.getArmState() == ArmState.Hold).setTimeout(3),
                    new IntakeCommandAction(new IntakeCommand(IntakeState.Defense))
                )
            )
        ));
    }
}
