package frc.robot.auto.actions;

import java.util.function.Supplier;

public class ConditionalAction extends Action {
    private final Supplier<Boolean> condition;

    private Action TrueAction;
    private Action FalseAction;

    private boolean valueAtStart;

    public ConditionalAction(Supplier<Boolean> condition) {
        this.condition = condition;
    }

    public ConditionalAction trueAction(Action trueAction) {
        TrueAction = trueAction;
        return this;
    }

    public ConditionalAction falseAction(Action falseAction) {
        FalseAction = falseAction;
        return this;
    }

    @Override
    protected void start() {
        valueAtStart = (condition.get() != null ? condition.get() : false);
    }

    @Override
    protected void run() {
        if(valueAtStart) {
            TrueAction.onLoop();
            setFinished(TrueAction.getEvaluatedDone());
        } else {
            FalseAction.onLoop();
            setFinished(FalseAction.getEvaluatedDone());
        }
    }

    @Override
    protected void done() {
        if(valueAtStart) {
            if(!TrueAction.getEvaluatedDone())
                TrueAction.done();
        } else {
            if(!FalseAction.getEvaluatedDone())
                FalseAction.done();
        }
    }
}
