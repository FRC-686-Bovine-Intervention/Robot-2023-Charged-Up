package frc.robot.auto.modes;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfiguration;
import frc.robot.auto.actions.ArmCommandAction;
import frc.robot.auto.actions.DriveOnChargeStationEdgeAction;
import frc.robot.auto.actions.DrivePercentAction;
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

public class OneSkipBalanceAuto extends AutoMode {
    private final ArmStatus armStatus = ArmStatus.getInstance();

    public OneSkipBalanceAuto(AutoConfiguration config) {
        
        Trajectory trajectory = AutoTrajectories.SkipBackward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];

        startConfiguration = new RobotConfiguration(trajectory.getInitialPose(), ArmPose.Preset.AUTO_START, ArmState.Hold);

        RamseteController ramseteController = new RamseteController(2, 0.7);

        addAction(new WaitUntilAction(() -> armStatus.getCurrentArmPose() == ArmPose.Preset.HOLD));
        addAction(new ArmCommandAction(new ArmCommand(ArmState.AlignNode).setTargetNode(config.startingPiece == GamePiece.Cube ? NodeEnum.TopCenter : (config.startingPosition == StartPosition.Loading ? NodeEnum.TopWall : NodeEnum.TopLoading))));
        addAction(new WaitUntilAction(() -> armStatus.getArmState() == ArmState.Adjust));
        addAction(new ArmCommandAction(new ArmCommand(ArmState.Release)));
        addAction(new WaitUntilAction(() -> armStatus.getClawGrabbing() == false));
        addAction(new RamseteFollowerAction(trajectory, ramseteController));
        addAction(new DriveOnChargeStationEdgeAction(true).setTimeout(3));
        addAction(new DrivePercentAction(12, -0.3));
        addAction(new DriverAssistCommandAction(new DriverAssistCommand(DriverAssistState.AutoBalance)));
    }
}
