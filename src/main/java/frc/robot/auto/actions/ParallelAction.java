package frc.robot.auto.actions;

public class ParallelAction extends Action {
    private final Action[] actions;

    public ParallelAction(Action... actions) {
        this.actions = actions;
    }

    @Override
    public void start() {}

    @Override
    public void run() {
        setFinished(true);
        int done = actions.length;
        for(Action action : actions) {
            action.onLoop();
            if(!action.getEvaluatedDone()) {
                done--;
                setFinished(false);
            }
        }
        System.out.println(done + " Parallel actions done");
    }

    @Override
    public void done() {
        for(Action action : actions) {
            if(!action.getEvaluatedDone())
                action.done();
        }
    }
}
