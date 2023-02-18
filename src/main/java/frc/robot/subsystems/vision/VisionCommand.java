package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionStatus.LimelightPipeline;

public class VisionCommand {
    public VisionCommand(){
        this(null);
    }
    public VisionCommand(LimelightPipeline targetPipeline){
        setTargetPipeline(targetPipeline);
    }

    private LimelightPipeline   targetPipeline = null;
    public LimelightPipeline    getTargetPipeline() {return targetPipeline;}
    public VisionCommand        setTargetPipeline(LimelightPipeline targetPipeline) {this.targetPipeline = targetPipeline; return this;}
}
