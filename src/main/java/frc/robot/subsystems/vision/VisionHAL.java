package frc.robot.subsystems.vision;

import frc.robot.subsystems.framework.HALBase;

public class VisionHAL extends HALBase {
    public static VisionHAL instance;
    public static VisionHAL getInstance(){
        if(instance == null){
            instance = new VisionHAL();
        }
        return instance;
    }
}
