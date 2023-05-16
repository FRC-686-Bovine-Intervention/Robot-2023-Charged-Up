package frc.robot.auto.actions;

import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionCommand;

public class IgnoreVisionAction extends Action {
    private final boolean ignoreVision;

    public IgnoreVisionAction(boolean ignoreVision) {
        this.ignoreVision = ignoreVision;
    }

    @Override
    protected void start() {
        Vision.getInstance().setVisionCommand(new VisionCommand().setIngoreVision(ignoreVision));
        setFinished(true);
    }

    @Override
    protected void run() {}

    @Override
    protected void done() {}
}
