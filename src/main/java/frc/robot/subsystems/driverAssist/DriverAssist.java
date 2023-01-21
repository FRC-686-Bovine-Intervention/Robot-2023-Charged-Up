package frc.robot.subsystems.driverAssist;

import frc.robot.subsystems.framework.SubsystemBase;

public class DriverAssist extends SubsystemBase {
    private static DriverAssist instance;
    public static DriverAssist getInstance(){if(instance == null){instance = new DriverAssist();}return instance;}

    private DriverAssist() {}

    @Override
    public void init()
    {
        loop = DriverAssistLoop.getInstance();
        status = DriverAssistStatus.getInstance();
    }

    private DriverAssistCommand command = new DriverAssistCommand();
    public DriverAssistCommand getCommand() {return command;}
    public DriverAssist setCommand(DriverAssistCommand command) {this.command = command; return this;}
}
