package frc.robot.subsystems.vision;

import frc.robot.subsystems.framework.SubsystemBase;

public class Vision extends SubsystemBase {
    private static Vision instance;
    public static Vision getInstance(){if(instance == null){instance = new Vision();}return instance;}    

    @Override
    public void init() {
        Loop = VisionLoop.getInstance();
        Status = VisionStatus.getInstance();
    }

    private VisionCommand   command = new VisionCommand();
    public VisionCommand    getVisionCommand()                              {return command;}
    public Vision           setVisionCommand(VisionCommand visionCommand)   {this.command = visionCommand; return this;}
    
}
