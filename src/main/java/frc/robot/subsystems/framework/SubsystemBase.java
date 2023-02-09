package frc.robot.subsystems.framework;

public abstract class SubsystemBase {
    public LoopBase Loop;
    public StatusBase Status;

    protected final void initialize() {
        init();
        boolean loopInitialized = false;
        boolean statusInitialized = false;

        if(Loop != null)
            loopInitialized = true;
        if(Status != null)
            statusInitialized = true;

        if(!(loopInitialized && statusInitialized))
        {
            throw new NullPointerException(
                this.getClass().getName() + " has not defined a " +
                (!loopInitialized ? "Loop " : "") +
                (!loopInitialized && !statusInitialized ? "nor a " : "") +
                (!statusInitialized ? "Status " : "") +
                "in the init() function\n");
        }
    }
    /**
     * DO NOT SET SUPER VARIABLES IN A CONSTRUCTOR<p>
     * Instead do it here<p>
     * Setting super variables in a constructor will cause a recursive .getInstance() loop
     */
    protected abstract void init();
}
