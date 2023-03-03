package frc.robot.auto.actions;

public abstract class ConditionalAction extends Action {
    private final Action action;

    public ConditionalAction(Action action) {
        this.action = action;
    }

    @Override
    protected void start() {
        if(!Condition())
            setFinished(true);
    }

    @Override
    protected void run() {
        action.onLoop();
        setFinished(action.getEvaluatedDone());
    }

    @Override
    protected void done() {
        if(!action.getEvaluatedDone())
            action.done();
    }
    
    public abstract boolean Condition();
}
