package frc.robot.auto.actions;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommand;

public class IntakeCommandAction extends Action {
    private final IntakeCommand command;

    public IntakeCommandAction(IntakeCommand command) {
        this.command = command;
    }

    @Override
    public void start() {
        Intake.getInstance().setCommand(command);
        setFinished(true);
    }

    @Override
    public void run() {}

    @Override
    public void done() {}
}
