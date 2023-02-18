package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.vision.VisionStatus.LimelightPipeline;

public class VisionLoop extends LoopBase {
    private static VisionLoop instance;
    public static VisionLoop getInstance(){if(instance == null){instance = new VisionLoop();}return instance;}

    private final VisionStatus status = VisionStatus.getInstance();

    private static final double kPipelineToggleDelay = 2;

    private VisionLoop() {Subsystem = Vision.getInstance();}

    private double pipelineStartTimestamp;
    
    @Override
    public void Update() {
        VisionCommand newCommand = status.getCommand();
        double currentTime = Timer.getFPGATimestamp();

        if(newCommand.getTargetPipeline() != null)
        {
            status.setTargetPipeline(newCommand.getTargetPipeline());
            pipelineStartTimestamp = currentTime;
        }
        else if(status.getTargetPipeline() == LimelightPipeline.Pole)
        {
            status.setTargetPipeline(LimelightPipeline.Cone);
        }

        if(status.getCurrentPipeline() == status.getTargetPipeline())
        {
            switch(status.getCurrentPipeline())
            {
                case Cone:  status.setTargetPipeline(LimelightPipeline.Cube);   break;
                case Cube:  status.setTargetPipeline(LimelightPipeline.Cone);   break;
                default:    break;
            }
            pipelineStartTimestamp = currentTime;
        }

        if(status.getTargetExists())
        {
            switch(status.getCurrentPipeline())
            {
                case Cone:
                    status.setLatestConeXAngle(status.getTargetXAngle());
                    status.setLatestConeYAngle(status.getTargetYAngle());
                    break;
                    
                case Cube:
                    status.setLatestCubeXAngle(status.getTargetXAngle());
                    status.setLatestCubeYAngle(status.getTargetYAngle());
                break;
    
                default: break;
            }
        }
    }

    @Override public void Enabled() {}
    @Override public void Disabled() {}
}
