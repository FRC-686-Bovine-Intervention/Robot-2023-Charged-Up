package frc.robot.auto.actions;

public class WaitAction extends Action {
    private final double waitTime;

    public WaitAction(double time) {
        this.waitTime = time;
    }

    @Override
    public void start() {}

    @Override
    public void run() {
        setFinished(actionTimer.hasElapsed(waitTime));
    }

    @Override
    public void done() {}
}
