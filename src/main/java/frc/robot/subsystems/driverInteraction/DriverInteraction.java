package frc.robot.subsystems.driverInteraction;

import frc.robot.subsystems.framework.SubsystemBase;

public class DriverInteraction extends SubsystemBase {
    private static DriverInteraction instance;
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;}

    @Override
    public void init() {
        Loop = DriverInteractionLoop.getInstance();
        Status = DriverInteractionStatus.getInstance();
    }
}
