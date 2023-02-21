package frc.robot.subsystems.framework;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;

public abstract class StatusBase implements LoggableInputs{

    public SubsystemBase Subsystem;

    public enum EnabledState
    {
        Starting(true),
        Enabled(false),
        Stopping(true),
        Disabled(false);
        public final boolean IsInitState;
        private EnabledState(boolean IsInitState)
        {
            this.IsInitState = IsInitState;
        }
    }
    public EnabledState Enabled = EnabledState.Disabled;

    protected GenericEntry EnabledEntry;
    private boolean EnabledSwitch = true;

    @Override
    public final void toLog(LogTable table)
    {
        table.put("Enabled Switch", EnabledSwitch);
        table.put("Enabled State", Enabled.name());
        exportToTable(table);
    }

    @Override
    public final void fromLog(LogTable table)
    {
        EnabledSwitch = table.getBoolean("Enabled Switch", true);
        switch(table.getString("Enabled State", "Default"))
        {
            case "Starting":
                Enabled = EnabledState.Starting;
            break;
            case "Enabled":
                Enabled = EnabledState.Enabled;
            break;
            case "Stopping":
                Enabled = EnabledState.Stopping;
            break;
            case "Disabled":
                Enabled = EnabledState.Disabled;
            break;
            default: break;
        }
        importFromTable(table);
    }

    protected final void runPreLoop()
    {
        if(EnabledEntry == null)
            EnabledSwitch = true;
        else
            EnabledSwitch = EnabledEntry.getBoolean(true);
        switch(Enabled)
        {
            case Starting:
                Enabled = EnabledState.Enabled;
            break;
            case Enabled:
                if(DriverStation.isDisabled() || !EnabledSwitch)
                    Enabled = EnabledState.Stopping;
            break;
            case Stopping:
                Enabled = EnabledState.Disabled;
            break;
            case Disabled:
                if(DriverStation.isEnabled() && EnabledSwitch)
                    Enabled = EnabledState.Starting;
            break;
        }
        updateInputs();
        Logger.getInstance().processInputs(this.getClass().getSimpleName(), this);
        processTable();
    }

    protected final void runPostLoop()
    {
        if(Subsystem == null)
            throw new NullPointerException(this.getClass().getName() + " has not defined the super variable Subsystem\n");
        String prefix = Subsystem.getClass().getSimpleName() + "/";
        Logger.getInstance().recordOutput(prefix + "Enabled Switch", EnabledSwitch);
        Logger.getInstance().recordOutput(prefix + "Enabled State", Enabled.name());
        processOutputs(Logger.getInstance(), prefix);
    }

    /**
     * First thing called in a loop<p>
     * Should be for reading raw inputs from the HAL
     */
    protected abstract void updateInputs();
    /**
     * Called directly after updateInputs if the code is not running in a simulation<p>
     * Should be for exporting raw inputs to a table
     * @param table - The {@link LogTable} to record to
     */
    protected abstract void exportToTable(LogTable table);
    /**
     * Called directly after updateInputs if the code is running in a simulation<p>
     * Should be for reading raw inputs from a table
     * @param table - The {@link LogTable} to pull from
     */
    protected abstract void importFromTable(LogTable table);
    /**
     * Called directly after the table methods<p>
     * Should be for updating "hardware" with the table data
     */
    protected abstract void processTable();
    /**
     * Called directly after the loop methods<p>
     * Should be sending loop outputs to the HAL and recording outputs to AdvantageKit
     * @param logger - How to record to AdvantageKit {@code logger.recordOutput(prefix + "Your value name here", value);}
     * @param prefix - Used to organize all subsystem outputs into their own folder
     */
    protected abstract void processOutputs(Logger logger, String prefix);
}
