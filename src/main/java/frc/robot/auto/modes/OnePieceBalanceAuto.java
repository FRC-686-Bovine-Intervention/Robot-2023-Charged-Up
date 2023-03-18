package frc.robot.auto.modes;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfiguration;
import frc.robot.auto.actions.ArmCommandAction;
import frc.robot.auto.actions.DriveOnChargeStationEdgeAction;
import frc.robot.auto.actions.DriverAssistCommandAction;
import frc.robot.auto.actions.RamseteFollowerAction;
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

public class OnePieceBalanceAuto extends AutoMode {
    private final ArmStatus armStatus = ArmStatus.getInstance();

    public OnePieceBalanceAuto(AutoConfiguration config) {
        
        Trajectory[] trajectories = new Trajectory[3];

        trajectories[0] = AutoTrajectories.SkipBackward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];

        startConfiguration = new RobotConfiguration(trajectories[0].getInitialPose(), ArmPose.Preset.AUTO_START, ArmState.Hold);

        RamseteController ramseteController = new RamseteController(2, 0.7);

        addAction(new WaitUntilAction(() -> armStatus.getCurrentArmPose() == ArmPose.Preset.DEFENSE));
        addAction(new ArmCommandAction(new ArmCommand(ArmState.Extend).setTargetNode(config.startingPiece == GamePiece.Cube ? NodeEnum.TopCenter : (config.startingPosition == StartPosition.Loading ? NodeEnum.TopWall : NodeEnum.TopLoading))));
        addAction(new WaitUntilAction(() -> armStatus.getArmState() == ArmState.Adjust));
        addAction(new ArmCommandAction(new ArmCommand(ArmState.Release)));
        addAction(new WaitUntilAction(() -> armStatus.getClawGrabbing() == false));
        addAction(new RamseteFollowerAction(trajectories[0], ramseteController));
        addAction(new DriveOnChargeStationEdgeAction(true));
        addAction(new DriverAssistCommandAction(new DriverAssistCommand(DriverAssistState.AutoBalance)));
    }
}
