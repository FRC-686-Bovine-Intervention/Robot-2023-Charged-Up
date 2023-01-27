package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

public class WaitAction implements Action {
    private final double endTime;

    public WaitAction(double time)
    {
        endTime = Timer.getFPGATimestamp() + time;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= endTime;
    }

    @Override public void start() {}
    @Override public void run() {}
    @Override public void done() {}
}
