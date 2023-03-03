package frc.robot.auto.actions;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmCommand;

public class ArmCommandAction extends Action {
    private final ArmCommand command;

    public ArmCommandAction(ArmCommand command) {
        this.command = command;
    }

    @Override
    public void start() {
        Arm.getInstance().setCommand(command);
        setFinished(true);
    }

    @Override
    public void run() {}

    @Override
    public void done() {}
}
