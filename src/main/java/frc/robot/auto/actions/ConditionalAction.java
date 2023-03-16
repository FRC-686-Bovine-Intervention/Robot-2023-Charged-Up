package frc.robot.auto.actions;

import java.util.function.Supplier;

public class ConditionalAction extends Action {
    private final Supplier<Boolean> condition;

    private final Action trueAction;
    private final Action falseAction;

    private boolean valueAtStart;

    public ConditionalAction(Supplier<Boolean> condition, Action trueAction) {this(condition, trueAction, null);}
    public ConditionalAction(Supplier<Boolean> condition, Action trueAction, Action falseAction) {
        this.condition = condition;
        this.trueAction = trueAction;
        this.falseAction = falseAction;
    }

    @Override
    protected void start() {
        valueAtStart = (condition.get() != null ? condition.get() : false);
    }

    @Override
    protected void run() {
        if(valueAtStart) {
            if(trueAction != null)
                trueAction.onLoop();
            setFinished(trueAction == null || trueAction.getEvaluatedDone());
        } else {
            if(falseAction != null)
                falseAction.onLoop();
            setFinished(falseAction == null || falseAction.getEvaluatedDone());
        }
    }

    @Override
    protected void done() {
        if(valueAtStart) {
            if(trueAction != null && !trueAction.getEvaluatedDone())
                trueAction.done();
        } else {
            if(falseAction != null && !falseAction.getEvaluatedDone())
                falseAction.done();
        }
    }
}
