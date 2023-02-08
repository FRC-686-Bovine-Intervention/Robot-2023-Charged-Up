package frc.robot.subsystems.arm;

import frc.robot.subsystems.framework.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm instance;
    public static Arm getInstance(){if(instance == null){instance = new Arm();}return instance;}

    @Override
    public void init() {
        Loop = ArmLoop.getInstance();
        Status = ArmStatus.getInstance();
    }
}
