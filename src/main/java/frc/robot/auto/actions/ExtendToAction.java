package frc.robot.auto.actions;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmCommand;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.arm.ArmStatus.NodeEnum;

public class ExtendToAction extends Action {
    private final Arm arm = Arm.getInstance();
    private final ArmStatus armStatus = ArmStatus.getInstance();

    private final NodeEnum node;

    public ExtendToAction(NodeEnum node) {
        this.node = node;
    }

    @Override
    protected void start() {
        arm.setCommand(new ArmCommand(ArmState.AlignNode).setTargetNode(node));
    }

    @Override
    protected void run() {
        setFinished(armStatus.getArmState() == ArmState.Adjust);
    }

    @Override
    protected void done() {
        
    }
    
}
