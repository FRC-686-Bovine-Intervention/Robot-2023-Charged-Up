package frc.robot.auto.actions;

import frc.robot.subsystems.driverAssist.DriverAssist;
import frc.robot.subsystems.driverAssist.DriverAssistCommand;

public class DriverAssistCommandAction extends Action {
    private final DriverAssistCommand command;
    
    public DriverAssistCommandAction(DriverAssistCommand command) {
        this.command = command;
    }

    @Override
    protected void start() {
        DriverAssist.getInstance().setCommand(command);
        setFinished(true);
    }

    @Override protected void run() {}
    @Override protected void done() {}
}
