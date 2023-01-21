package frc.robot.subsystems.framework;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.oblarg.oblog.Loggable;

public abstract class StatusBase implements LoggableInputs, Loggable{

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
    public abstract void exportToTable(LogTable table);

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
    public abstract void importFromTable(LogTable table);

    public final void runPreLoop()
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
    }
    public abstract void updateInputs();

    public final void runPostLoop()
    {
        String prefix = Subsystem.getClass().getSimpleName() + "/";
        Logger.getInstance().recordOutput(prefix + "Enabled Switch", EnabledSwitch);
        Logger.getInstance().recordOutput(prefix + "Enabled State", Enabled.name());
        recordOutputs(Logger.getInstance(), prefix);
    }
    public abstract void recordOutputs(Logger logger, String prefix);
}
