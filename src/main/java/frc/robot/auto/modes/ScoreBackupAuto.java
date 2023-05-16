package frc.robot.auto.modes;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfiguration;
import frc.robot.auto.actions.ArmCommandAction;
import frc.robot.auto.actions.DrivePercentAction;
import frc.robot.auto.actions.WaitUntilAction;
import frc.robot.auto.autoManager.AutoConfiguration;
import frc.robot.auto.autoManager.AutoConfiguration.GamePiece;
import frc.robot.auto.autoManager.AutoConfiguration.StartPosition;
import frc.robot.subsystems.arm.ArmCommand;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.arm.ArmStatus.NodeEnum;

public class ScoreBackupAuto extends AutoMode {
    private final ArmStatus armStatus = ArmStatus.getInstance();
    public ScoreBackupAuto(AutoConfiguration config) {
        Trajectory trajectory = AutoTrajectories.SkipBackward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];
        
        startConfiguration = new RobotConfiguration(trajectory.getInitialPose(), ArmPose.Preset.AUTO_START, ArmState.Hold);

        addAction(new WaitUntilAction(() -> armStatus.getCurrentArmPose() == ArmPose.Preset.HOLD));
        addAction(new ArmCommandAction(new ArmCommand(ArmState.AlignNode).setTargetNode(config.startingPiece == GamePiece.Cube ? NodeEnum.TopCenter : (config.startingPosition == StartPosition.Loading ? NodeEnum.TopWall : NodeEnum.TopLoading))));
        addAction(new WaitUntilAction(() -> armStatus.getArmState() == ArmState.Adjust));
        addAction(new ArmCommandAction(new ArmCommand(ArmState.Release)));
        addAction(new WaitUntilAction(() -> armStatus.getClawGrabbing() == false));
        addAction(new DrivePercentAction(144, -0.5).setTimeout(5));
    }
}
