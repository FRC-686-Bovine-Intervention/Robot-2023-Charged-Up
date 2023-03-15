package frc.robot.auto.autoManager;

import java.lang.reflect.InvocationTargetException;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotConfiguration;
import frc.robot.auto.AutoConfiguration;
import frc.robot.auto.modes.AutoMode;
import frc.robot.auto.modes.BlankAutoMode;
import frc.robot.auto.modes.DriveStraightAuto;
import frc.robot.auto.modes.OnePieceBalanceAuto;
import frc.robot.auto.modes.RamseteFollowerTestAuto;
import frc.robot.subsystems.framework.StatusBase;

public class AutoManagerStatus extends StatusBase {
    private static AutoManagerStatus instance;
    public static AutoManagerStatus getInstance(){if(instance == null){instance = new AutoManagerStatus();}return instance;}

    public enum AutoModesEnum{
        WallSideOnePieceBalance("One Piece Balance", OnePieceBalanceAuto.class),
        WallSideTwoPiece("Two Piece", OnePieceBalanceAuto.class),
        DriveStraightAuto("Drive Straight Test", DriveStraightAuto.class),
        RamseteFollowerTest("Ramsete Follower Test", RamseteFollowerTestAuto.class),
        Blank("Blank Mode Test", BlankAutoMode.class),
        ;
        public final String autoName;
        public final Class<? extends AutoMode> autoMode;
        AutoModesEnum(Class<? extends AutoMode> autoMode) {this(autoMode.getSimpleName(), autoMode);}
        AutoModesEnum(String autoName, Class<? extends AutoMode> autoMode)
        {
            this.autoName = autoName;
            this.autoMode = autoMode;
        }
    }

    private final SendableChooser<AutoModesEnum> autoChooser = new SendableChooser<AutoModesEnum>();

    private final ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    private final GenericEntry initialDelayEntry = tab.add("Initial Delay (Sec)", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

    private AutoManagerStatus()
    {
        Subsystem = AutoManager.getInstance();
        for(int i = 0; i < AutoModesEnum.values().length; i++)
        {
            AutoModesEnum mode = AutoModesEnum.values()[i];
            if(i == 0)
                autoChooser.setDefaultOption(mode.autoName, mode);
            else
                autoChooser.addOption(mode.autoName, mode);
        }
        tab.add("Auto Mode", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    private double NT_InitialDelay;
    public double getNT_InitialDelay()                                      {return NT_InitialDelay;}
    private AutoManagerStatus setNT_InitialDelay(double NT_InitialDelay)    {this.NT_InitialDelay = NT_InitialDelay; return this;}

    private double initialDelay;
    public double getInitialDelay()                                 {return initialDelay;}
    public AutoManagerStatus setInitialDelay(double initialDelay)   {this.initialDelay = initialDelay; return this;}

    private AutoModesEnum NT_SelectedAutoMode;
    public AutoModesEnum getNT_SelectedAutoMode()                                       {return NT_SelectedAutoMode;}
    private AutoManagerStatus setNT_SelectedAutoMode(AutoModesEnum NT_SelectedAutoMode) {this.NT_SelectedAutoMode = NT_SelectedAutoMode; return this;}

    private AutoModesEnum selectedAutoMode;
    public AutoModesEnum getSelectedAutoMode()                                      {return selectedAutoMode;}
    public AutoManagerStatus setSelectedAutoMode(AutoModesEnum selectedAutoMode)    {this.selectedAutoMode = selectedAutoMode; return this;}
    public Class<? extends AutoMode> getAutomode()                                  {return selectedAutoMode.autoMode;}
    public AutoMode getNewAutomode() {
        try {
            try {
                return getAutomode().getConstructor(AutoConfiguration.class).newInstance();
            } catch(NoSuchMethodException e) {
                return getAutomode().getConstructor().newInstance();
            }
        } catch (InvocationTargetException e) {
            
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }
    private AutoMode currentAutoMode;
    public AutoMode getCurrentAutoMode()                            {return currentAutoMode;}
    public AutoManagerStatus setCurrentAutoMode(AutoMode autoMode)  {this.currentAutoMode = autoMode; return this;}
    
    private boolean autoRunning;
    public boolean getAutoRunning() {return autoRunning;}
    public AutoManagerStatus setAutoRunning(boolean autoRunning) {this.autoRunning = autoRunning; return this;}

    private int actionIndex = -1;
    public int getActionIndex() {return actionIndex;}
    public AutoManagerStatus setActionIndex(int actionIndex) {this.actionIndex = actionIndex; return this;}
    public AutoManagerStatus incrementActionIndex(int increment) {this.actionIndex += increment; return this;}

    private AutoConfiguration   autoConfiguration = new AutoConfiguration();
    public AutoConfiguration    getAutoConfiguration()                                      {return autoConfiguration;}
    protected AutoManagerStatus setAutoConfiguration(AutoConfiguration autoConfiguration)   {this.autoConfiguration = autoConfiguration; return this;}
    
    @Override
    protected void updateInputs() {
        setNT_InitialDelay(initialDelayEntry.getDouble(NT_InitialDelay));
        setNT_SelectedAutoMode(autoChooser.getSelected());
    }

    @Override
    protected void exportToTable(LogTable table) {
        table.put("Initial Delay", NT_InitialDelay);
        table.put("Selected Auto Mode", NT_SelectedAutoMode != null ? NT_SelectedAutoMode.name() : null);
    }

    @Override
    protected void importFromTable(LogTable table) {
        setNT_InitialDelay(table.getDouble("Initial Delay", NT_InitialDelay));
        String NTModeName = table.getString("Selected AutoMode", NT_SelectedAutoMode != null ? NT_SelectedAutoMode.name() : null);
        AutoModesEnum NTAutoMode = null;
        for(AutoModesEnum mode : AutoModesEnum.values())
        {
            if(mode.name() == NTModeName)
            {
                NTAutoMode = mode;
                break;
            }
        }
        setNT_SelectedAutoMode(NTAutoMode);
    }

    @Override
    protected void processOutputs(Logger logger, String prefix) {
        logger.recordOutput(prefix + "Initial Delay", initialDelay);
        logger.recordOutput(prefix + "Selected Auto Mode", selectedAutoMode != null ? selectedAutoMode.name() : null);
        logger.recordOutput(prefix + "Auto Mode Class", getAutomode() != null ? getAutomode().getSimpleName() : null);
        logger.recordOutput(prefix + "Auto Running", autoRunning);
        logger.recordOutput(prefix + "Action Index", actionIndex);
    }

    @Override protected void processTable() {}
    @Override protected void loadConfiguration(RobotConfiguration configuration) {}
}
