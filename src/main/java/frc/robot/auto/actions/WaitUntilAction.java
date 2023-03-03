package frc.robot.auto.actions;

public abstract class WaitUntilAction extends Action {

    @Override
    protected void start() {}

    @Override
    protected void run() {
        setFinished(Condition());
    }

    @Override
    protected void done() {}
    
    public abstract boolean Condition();
}
