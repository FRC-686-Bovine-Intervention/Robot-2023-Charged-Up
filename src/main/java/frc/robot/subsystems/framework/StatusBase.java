package frc.robot.subsystems.framework;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfiguration;
import frc.robot.subsystems.arm.Arm;

public abstract class StatusBase implements LoggableInputs{

    public SubsystemBase Subsystem;

    public enum EnabledStateEnum
    {
        Starting    (true, true),
        Enabled     (true, false),
        Stopping    (false, true),
        Disabled    (false, false);
        public final boolean IsEnabled;
        public final boolean IsInitState;
        private EnabledStateEnum(boolean IsEnabled, boolean IsInitState)
        {
            this.IsEnabled = IsEnabled;
            this.IsInitState = IsInitState;
        }
    }
    public EnabledStateEnum EnabledState = EnabledStateEnum.Disabled;

    protected GenericEntry EnabledEntry;
    private boolean EnabledSwitch = true;

    @Override
    public final void toLog(LogTable table)
    {
        table.put("Enabled Switch", EnabledSwitch);
        table.put("Enabled State", EnabledState.name());
        exportToTable(table);
    }

    @Override
    public final void fromLog(LogTable table)
    {
        EnabledSwitch = table.getBoolean("Enabled Switch", true);
        switch(table.getString("Enabled State", "Default"))
        {
            case "Starting":
                EnabledState = EnabledStateEnum.Starting;
            break;
            case "Enabled":
                EnabledState = EnabledStateEnum.Enabled;
            break;
            case "Stopping":
                EnabledState = EnabledStateEnum.Stopping;
            break;
            case "Disabled":
                EnabledState = EnabledStateEnum.Disabled;
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
        switch(EnabledState)
        {
            case Starting:
                EnabledState = EnabledStateEnum.Enabled;
            break;
            case Enabled:
                if(DriverStation.isDisabled() || !EnabledSwitch)
                    EnabledState = EnabledStateEnum.Stopping;
            break;
            case Stopping:
                EnabledState = EnabledStateEnum.Disabled;
            break;
            case Disabled:
                if(DriverStation.isEnabled() && EnabledSwitch)
                    EnabledState = EnabledStateEnum.Starting;
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
        if(DriverStation.isEnabled() || Subsystem.getClass() == Arm.class) {
            Logger.getInstance().recordOutput(prefix + "Enabled Switch", EnabledSwitch);
            Logger.getInstance().recordOutput(prefix + "Enabled State", EnabledState.name());
            processOutputs(Logger.getInstance(), prefix);
        }
    }

    /**
     * 
     * @param configuration - The configuration to load
     */
    protected abstract void loadConfiguration(RobotConfiguration configuration);
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
