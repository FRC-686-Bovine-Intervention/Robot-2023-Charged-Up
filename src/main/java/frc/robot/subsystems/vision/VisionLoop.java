package frc.robot.subsystems.vision;

import frc.robot.subsystems.framework.LoopBase;

public class VisionLoop extends LoopBase {
    public static VisionLoop instance;
    public static VisionLoop getInstance(){
        if(instance == null){
            instance = new VisionLoop();
        }
        return instance;
    }

    private VisionLoop()
    {
        Subsystem = Vision.getInstance();
    }


    @Override
    public void Enabled() {
        
    }

    @Override
    public void Disabled() {
        
        
    }

    @Override
    public void Update() {
        // TODO Auto-generated method stub
        
    }
    
}
