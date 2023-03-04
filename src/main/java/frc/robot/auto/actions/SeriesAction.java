package frc.robot.auto.actions;

public class SeriesAction extends Action {
    private final Action[] actions;

    public SeriesAction(Action... actions) {
        this.actions = actions;
    }

    private int actionIndex = 0;

    @Override
    protected void start() {}

    @Override
    protected void run() {
        actions[actionIndex].onLoop();
        if(actions[actionIndex].getEvaluatedDone())
            actionIndex++;
        setFinished(actionIndex >= actions.length);
    }

    @Override
    protected void done() {
        if(!actions[actionIndex].getEvaluatedDone())
            actions[actionIndex].done();
    }
}
