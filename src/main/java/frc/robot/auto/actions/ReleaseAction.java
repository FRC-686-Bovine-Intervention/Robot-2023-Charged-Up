package frc.robot.auto.actions;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmCommand;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.arm.ArmStatus.ArmState;

public class ReleaseAction extends Action {
    private final Arm arm = Arm.getInstance();
    private final ArmStatus armStatus = ArmStatus.getInstance();

    @Override
    protected void start() {
        arm.setCommand(new ArmCommand(ArmState.Release));
    }

    @Override
    protected void run() {
        setFinished(armStatus.getTargetArmPose() == ArmPose.Preset.HOLD);
    }

    @Override
    protected void done() {
        
    }
    
}
