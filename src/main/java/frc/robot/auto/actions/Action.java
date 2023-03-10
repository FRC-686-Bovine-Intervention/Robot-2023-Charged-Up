package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public abstract class Action {
    public final Timer actionTimer = new Timer();

    public final void onLoop() {
        if(!started) {
            setStarted(true);
            actionTimer.start();
            start();
        }
        if(!finished) {
            run();
            if(timeout > 0 && actionTimer.hasElapsed(timeout)) {
                setFinished(true);
                DriverStation.reportWarning(this.getClass().getSimpleName() + " timed out at " + timeout + " seconds", true);
            }
        }
        if(finished && !evalutatedDone) {
            setEvaluatedDone(true);
            actionTimer.stop();
            done();
        }
    }

    private boolean         started = false;
    public final boolean    getStarted()                {return started;}
    private final void      setStarted(boolean started) {this.started = started;}

    private boolean         finished = false;
    public final boolean    getFinished()                   {return finished;}
    protected final void    setFinished(boolean finished)   {this.finished = finished;}

    private boolean         evalutatedDone = false;
    public final boolean    getEvaluatedDone()                      {return evalutatedDone;} 
    private final void      setEvaluatedDone(boolean evaluatedDone) {this.evalutatedDone = evaluatedDone;}

    private double      timeout = 0;
    public final double getTimeout()                {return timeout;}
    public final Action setTimeout(double timeout)  {this.timeout = timeout; return this;}

    /**
     * Run code once when the action is started, for set up
     */
    protected abstract void start();

    /**
     * Called by runAction in AutoModeBase iteratively until isFinished returns
     * true. Iterative logic lives in this method
     */
    protected abstract void run();

    /**
     * Run code once when the action finishes, usually for clean up
     */
    protected abstract void done();
}
