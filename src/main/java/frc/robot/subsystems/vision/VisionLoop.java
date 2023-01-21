package frc.robot.subsystems.vision;

import frc.robot.subsystems.framework.LoopBase;

public class VisionLoop extends LoopBase {
    private static VisionLoop instance;
    public static VisionLoop getInstance(){if(instance == null){instance = new VisionLoop();}return instance;}

    // private final VisionHAL HAL = VisionHAL.getInstance();
    // private final VisionStatus status = VisionStatus.getInstance();

    private VisionLoop()
    {
        Subsystem = Vision.getInstance();
    }
    
    @Override
    public void Update() {
        
    }

    @Override public void Enabled() {}
    @Override public void Disabled() {}
}
