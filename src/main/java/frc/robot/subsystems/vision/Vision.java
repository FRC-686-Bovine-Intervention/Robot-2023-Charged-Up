package frc.robot.subsystems.vision;

import frc.robot.subsystems.framework.SubsystemBase;

public class Vision extends SubsystemBase {
    private static Vision instance;
    public static Vision getInstance(){if(instance == null){instance = new Vision();}return instance;}    

    @Override
    public void init() {
        loop = VisionLoop.getInstance();
        status = VisionStatus.getInstance();
    }
    
}
