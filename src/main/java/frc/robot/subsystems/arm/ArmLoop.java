package frc.robot.subsystems.arm;

import frc.robot.subsystems.framework.LoopBase;

public class ArmLoop extends LoopBase {
    private static ArmLoop instance;
    public static ArmLoop getInstance(){if(instance == null) instance = new ArmLoop(); return instance;}

    private ArmLoop() {Subsystem = Arm.getInstance();}

    @Override
    protected void Enabled() {
        
    }
    @Override
    protected void Disabled() {
        
    }
    @Override
    protected void Update() {
        
    }
}
