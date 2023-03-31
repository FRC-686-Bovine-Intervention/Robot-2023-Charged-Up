package frc.robot.auto.autoManager;

import java.lang.reflect.InvocationTargetException;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotConfiguration;
import frc.robot.auto.autoManager.AutoConfiguration.GamePiece;
import frc.robot.auto.autoManager.AutoConfiguration.StartPosition;
import frc.robot.auto.modes.AutoMode;
import frc.robot.auto.modes.DriveStraightAuto;
import frc.robot.auto.modes.OneGrabBalanceAuto;
import frc.robot.auto.modes.OneSkipBalanceAuto;
import frc.robot.auto.modes.ScoreBackupAuto;
import frc.robot.auto.modes.TwoPieceAuto;
import frc.robot.subsystems.framework.StatusBase;

public class AutoManagerStatus extends StatusBase {
    private static AutoManagerStatus instance;
    public static AutoManagerStatus getInstance(){if(instance == null){instance = new AutoManagerStatus();}return instance;}

    public enum AutoModesEnum{
        OneSkipBalance("One Piece Skip Balance", OneSkipBalanceAuto.class),
        TwoPiece("Two Piece", TwoPieceAuto.class),
        OneGrabBalance("One Piece Grab Balance", OneGrabBalanceAuto.class),
        ScoreBackup("Score Backup", ScoreBackupAuto.class),
        DriveStraightAuto("Drive Straight Test", DriveStraightAuto.class),
        // RamseteFollowerTest("Ramsete Follower Test", RamseteFollowerTestAuto.class),
        // Blank("Blank Mode Test", BlankAutoMode.class),
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

    private final SendableChooser<AutoModesEnum> modeChooser = new SendableChooser<AutoModesEnum>();
    private final SendableChooser<StartPosition> poseChooser = new SendableChooser<StartPosition>();
    private final SendableChooser<GamePiece> startingPieceChooser = new SendableChooser<GamePiece>();
    private final SendableChooser<GamePiece> piece0Chooser = new SendableChooser<GamePiece>();
    private final SendableChooser<GamePiece> piece1Chooser = new SendableChooser<GamePiece>();
    private final SendableChooser<GamePiece> piece2Chooser = new SendableChooser<GamePiece>();
    private final SendableChooser<GamePiece> piece3Chooser = new SendableChooser<GamePiece>();

    private final ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    private final GenericEntry initialDelayEntry = tab.add("Initial Delay (Sec)", 0).withWidget(BuiltInWidgets.kTextView).withPosition(0,3).withSize(2, 1).getEntry();

    private AutoManagerStatus()
    {
        Subsystem = AutoManager.getInstance();
        for(int i = 0; i < AutoModesEnum.values().length; i++) {
            AutoModesEnum mode = AutoModesEnum.values()[i];
            if(i == 0)
                modeChooser.setDefaultOption(mode.autoName, mode);
            else
                modeChooser.addOption(mode.autoName, mode);
        }
        tab.add("Auto Mode", modeChooser).withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
        for(int i = 0; i < StartPosition.values().length; i++) {
            StartPosition pose = StartPosition.values()[i];
            if(i == 0)
                poseChooser.setDefaultOption(pose.name(), pose);
            else
                poseChooser.addOption(pose.name(), pose);
        }
        tab.add("Starting Pose", poseChooser).withPosition(0, 1).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
        for(int i = 0; i < GamePiece.values().length; i++) {
            GamePiece piece = GamePiece.values()[i];
            if(i == 0) {
                startingPieceChooser.setDefaultOption(piece.name(), piece);
                piece0Chooser.setDefaultOption(piece.name(), piece);
                piece1Chooser.setDefaultOption(piece.name(), piece);
                piece2Chooser.setDefaultOption(piece.name(), piece);
                piece3Chooser.setDefaultOption(piece.name(), piece);
            } else {
                startingPieceChooser.addOption(piece.name(), piece);
                piece0Chooser.addOption(piece.name(), piece);
                piece1Chooser.addOption(piece.name(), piece);
                piece2Chooser.addOption(piece.name(), piece);
                piece3Chooser.addOption(piece.name(), piece);
            }
        }
        tab.add("Starting Piece", startingPieceChooser).withPosition(0, 2).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
        tab.add("Staged Piece 0", piece0Chooser).withPosition(2, 3).withSize(1, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
        tab.add("Staged Piece 1", piece1Chooser).withPosition(2, 2).withSize(1, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
        tab.add("Staged Piece 2", piece2Chooser).withPosition(2, 1).withSize(1, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
        tab.add("Staged Piece 3", piece3Chooser).withPosition(2, 0).withSize(1, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    private double NT_InitialDelay;
    public double getNT_InitialDelay()                                      {return NT_InitialDelay;}
    private AutoManagerStatus setNT_InitialDelay(double NT_InitialDelay)    {this.NT_InitialDelay = NT_InitialDelay; return this;}

    private StartPosition       NT_StartingPose;
    public StartPosition        getNT_StartingPose()                                {return NT_StartingPose;}
    private AutoManagerStatus   setNT_StartingPose(StartPosition NT_StartingPose)   {this.NT_StartingPose = NT_StartingPose; return this;}

    private GamePiece           NT_StartingPiece;
    public GamePiece            getNT_StartingPiece()                           {return NT_StartingPiece;}
    private AutoManagerStatus   setNT_StartingPiece(GamePiece NT_StartingPiece) {this.NT_StartingPiece = NT_StartingPiece; return this;}

    private GamePiece           NT_StagedPiece0;
    public GamePiece            getNT_StagedPiece0()                            {return NT_StagedPiece0;}
    private AutoManagerStatus   setNT_StagedPiece0(GamePiece NT_StagedPiece0)   {this.NT_StagedPiece0 = NT_StagedPiece0; return this;}

    private GamePiece           NT_StagedPiece1;
    public GamePiece            getNT_StagedPiece1()                            {return NT_StagedPiece1;}
    private AutoManagerStatus   setNT_StagedPiece1(GamePiece NT_StagedPiece1)   {this.NT_StagedPiece1 = NT_StagedPiece1; return this;}

    private GamePiece           NT_StagedPiece2;
    public GamePiece            getNT_StagedPiece2()                            {return NT_StagedPiece2;}
    private AutoManagerStatus   setNT_StagedPiece2(GamePiece NT_StagedPiece2)   {this.NT_StagedPiece2 = NT_StagedPiece2; return this;}

    private GamePiece           NT_StagedPiece3;
    public GamePiece            getNT_StagedPiece3()                            {return NT_StagedPiece3;}
    private AutoManagerStatus   setNT_StagedPiece3(GamePiece NT_StagedPiece3)   {this.NT_StagedPiece3 = NT_StagedPiece3; return this;}

    private AutoModesEnum NT_SelectedAutoMode;
    public AutoModesEnum getNT_SelectedAutoMode()                                       {return NT_SelectedAutoMode;}
    private AutoManagerStatus setNT_SelectedAutoMode(AutoModesEnum NT_SelectedAutoMode) {this.NT_SelectedAutoMode = NT_SelectedAutoMode; return this;}

    private AutoModesEnum selectedAutoMode;
    public AutoModesEnum getSelectedAutoMode()                                      {return selectedAutoMode;}
    public AutoManagerStatus setSelectedAutoMode(AutoModesEnum selectedAutoMode)    {this.selectedAutoMode = selectedAutoMode; return this;}
    public Class<? extends AutoMode> getAutomode()                                  {return (getSelectedAutoMode() != null ? getSelectedAutoMode().autoMode : null);}
    public AutoMode getNewAutomode() {
        try {
            try {
                return getAutomode().getConstructor(AutoConfiguration.class).newInstance(getAutoConfiguration());
            } catch(NoSuchMethodException e) {
                System.out.println("No configuration constructor");
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
        setNT_SelectedAutoMode(modeChooser.getSelected());
        setNT_StartingPose(poseChooser.getSelected());
        setNT_StartingPiece(startingPieceChooser.getSelected());
        setNT_StagedPiece0(piece0Chooser.getSelected());
        setNT_StagedPiece1(piece1Chooser.getSelected());
        setNT_StagedPiece2(piece2Chooser.getSelected());
        setNT_StagedPiece3(piece3Chooser.getSelected());
    }

    @Override
    protected void exportToTable(LogTable table) {
        table.put("Initial Delay", NT_InitialDelay);
        table.put("Selected Auto Mode", NT_SelectedAutoMode != null ? NT_SelectedAutoMode.name() : null);
        table.put("Starting Pose", NT_StartingPose != null ? NT_StartingPose.name() : null);
        table.put("Starting Piece", NT_StartingPiece != null ? NT_StartingPiece.name() : null);
        table.put("Staged Piece 0", NT_StagedPiece0 != null ? NT_StagedPiece0.name() : null);
        table.put("Staged Piece 1", NT_StagedPiece1 != null ? NT_StagedPiece1.name() : null);
        table.put("Staged Piece 2", NT_StagedPiece2 != null ? NT_StagedPiece2.name() : null);
        table.put("Staged Piece 3", NT_StagedPiece3 != null ? NT_StagedPiece3.name() : null);
    }

    @Override
    protected void importFromTable(LogTable table) {
        setNT_InitialDelay(table.getDouble("Initial Delay", NT_InitialDelay));
        setNT_SelectedAutoMode(AutoModesEnum.valueOf(table.getString("Selected AutoMode", NT_SelectedAutoMode != null ? NT_SelectedAutoMode.name() : null)));
        setNT_StartingPose(StartPosition.valueOf(table.getString("Starting Pose", NT_StartingPose != null ? NT_StartingPose.name() : null)));
        setNT_StartingPiece(GamePiece.valueOf(table.getString("Starting Piece", NT_StartingPiece != null ? NT_StartingPiece.name() : null)));
        setNT_StagedPiece0(GamePiece.valueOf(table.getString("Staged Piece 0", NT_StagedPiece0 != null ? NT_StagedPiece0.name() : null)));
        setNT_StagedPiece1(GamePiece.valueOf(table.getString("Staged Piece 1", NT_StagedPiece1 != null ? NT_StagedPiece1.name() : null)));
        setNT_StagedPiece2(GamePiece.valueOf(table.getString("Staged Piece 2", NT_StagedPiece2 != null ? NT_StagedPiece2.name() : null)));
        setNT_StagedPiece3(GamePiece.valueOf(table.getString("Staged Piece 3", NT_StagedPiece3 != null ? NT_StagedPiece3.name() : null)));
    }

    @Override protected void processTable() {}

    @Override
    protected void processOutputs(Logger logger, String prefix) {
        // TODO: LESS LOGGING
        // autoConfiguration.log(logger, prefix +  "Auto Configuration");
        // logger.recordOutput(prefix + "Selected Auto Mode", getSelectedAutoMode() != null ? getSelectedAutoMode().name() : null);
        // logger.recordOutput(prefix + "Auto Mode Class", getAutomode() != null ? getAutomode().getSimpleName() : null);
        // logger.recordOutput(prefix + "Auto Running", autoRunning);
        // logger.recordOutput(prefix + "Action Index", actionIndex);
    }

    @Override protected void loadConfiguration(RobotConfiguration configuration) {}
}
