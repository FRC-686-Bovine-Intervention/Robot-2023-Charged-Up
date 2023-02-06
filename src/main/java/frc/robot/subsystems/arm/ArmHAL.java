package frc.robot.subsystems.arm;

public class ArmHAL {
    private static ArmHAL instance;
    public static ArmHAL getInstance(){if(instance == null) instance = new ArmHAL(); return instance;}
}
