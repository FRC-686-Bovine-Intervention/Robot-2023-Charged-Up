package frc.robot.auto.actions;

import java.util.function.Supplier;

public class WaitUntilAction extends Action {
    private final Supplier<Boolean> condition;

    public WaitUntilAction(Supplier<Boolean> condition) {
        this.condition = condition;
    }

    @Override
    protected void start() {}

    @Override
    protected void run() {
        setFinished(condition.get() != null ? condition.get() : false);
    }

    @Override
    protected void done() {}
}
