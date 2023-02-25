package frc.robot.subsystems.framework;

import frc.robot.subsystems.framework.StatusBase.EnabledState;

public abstract class LoopBase {
    //TODO: Update documentation
    /**
     * <h3>MUST BE SET IN SUBCLASSES</h3><p>
     * Allows the default enabled status code to run<p>
     * Recommended that you set it to a subclass of {@link SubsystemBase}, but can be set to {@link SubsystemBase} as a default
     */
    protected SubsystemBase Subsystem;

    public final void onStart() {onEverything();}

    public final void onLoop() {onEverything();}

    public final void onStop() {onEverything(EnabledState.Stopping);}

    private void onEverything() {onEverything(null);}
    private void onEverything(EnabledState state)
    {
        if(Subsystem == null)
            throw new NullPointerException(this.getClass().getName() + " has not defined the super variable Subsystem\n");

        if(state != null)
            Subsystem.Status.Enabled = state;

        Update();
        switch(Subsystem.Status.Enabled)
        {
            case Starting:
            case Enabled:
                Enabled();
            break;
            case Stopping:
            case Disabled:
                Disabled();
            break;
        }
    }

    /**
     * <h3>Runs every loop tick when the subsystem is enabled</h3><p>
     * All code that should run when the subsystem is enabled goes here<p>
     * This should be for actuating motors and other objects on the robot
     */
    protected abstract void Enabled();
    /**
     * <h3>Runs every loop tick when the subsystem is disabled</h3><p>
     * All code that should run when the system is disabled goes here<p>
     * This should be for stopping objects and reseting variables
     */
    protected abstract void Disabled();
    /**
     * <h3>Runs every loop tick</h3><p>
     * All code that should run regardless of the system status goes here<p>
     * This should be for things like updating Shuffleboard
     */
    protected abstract void Update();
}
