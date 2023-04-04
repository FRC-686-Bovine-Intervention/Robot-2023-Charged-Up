package frc.robot.auto.actions;

import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.odometry.OdometryCommand;

public class IgnoreVisionAction extends Action {
    private final boolean ignoreVision;

    public IgnoreVisionAction(boolean ignoreVision) {
        this.ignoreVision = ignoreVision;
    }

    @Override
    protected void start() {
        Odometry.getInstance().setCommand(new OdometryCommand().setIngoreVision(ignoreVision));
        setFinished(true);
    }

    @Override
    protected void run() {}

    @Override
    protected void done() {}
}
