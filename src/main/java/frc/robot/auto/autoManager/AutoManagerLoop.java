package frc.robot.auto.autoManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.RamseteFollowerAction;
import frc.robot.subsystems.framework.LoopBase;

public class AutoManagerLoop extends LoopBase {
    private static AutoManagerLoop instance;
    public static AutoManagerLoop getInstance(){if(instance == null){instance = new AutoManagerLoop();}return instance;}

    private final AutoManagerStatus status = AutoManagerStatus.getInstance();

    private AutoManagerLoop() {Subsystem = AutoManager.getInstance();}

    private int actionIndex = -1;
    private int prevActionIndex = -1;
    private double startTime = 0;

    @Override
    protected void Enabled() {
        if(!DriverStation.isAutonomous())
            return;
        if(status.Enabled.IsInitState)
        {
            status.setAutoRunning(true);
            actionIndex = -1;
            startTime = Timer.getFPGATimestamp();
        }
        if(actionIndex < 0)
        {
            if(Timer.getFPGATimestamp() - startTime >= status.getInitialDelay())
            {
                actionIndex = 0;
            }
        }
        else
        {
            if(!status.getAutoRunning())
                return;
            Action action = status.getAutomode().actionList.get(actionIndex);
            status.setActionTrajectory(action.getClass() == RamseteFollowerAction.class ? ((RamseteFollowerAction)action).controller.getTrajectory() : null);
            if(actionIndex != prevActionIndex)
                action.start();
            action.run();
            prevActionIndex = actionIndex;
            if(action.isFinished())
            {
                action.done();
                actionIndex++;
            }
            if(actionIndex >= status.getAutomode().actionList.size())
            {
                status.setAutoRunning(false);
                System.out.println(status.getSelectedAutoMode().autoName + " finished");
            }
        }
        status.setActionIndex(actionIndex);
    }
    @Override
    protected void Disabled() {
        status.setInitialDelay(status.getNT_InitialDelay());
        status.setSelectedAutoMode(status.getNT_SelectedAutoMode());
    }
    @Override
    protected void Update() {
        
    }
}
