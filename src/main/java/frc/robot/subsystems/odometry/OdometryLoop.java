package frc.robot.subsystems.odometry;

import frc.robot.subsystems.framework.LoopBase;

public class OdometryLoop extends LoopBase {
    private static OdometryLoop instance;
    public static OdometryLoop getInstance(){if(instance == null){instance = new OdometryLoop();}return instance;}
    
    @Override
    public void Update() {
        
    }
    
    @Override public void Enabled() {}
    @Override public void Disabled() {}
}
