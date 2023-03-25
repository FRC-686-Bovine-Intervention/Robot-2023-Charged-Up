package frc.robot.auto.autoManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.actions.Action;
import frc.robot.auto.autoManager.AutoConfiguration.GamePiece;
import frc.robot.subsystems.arm.ArmTrajectory;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.framework.SubsystemController;

public class AutoManagerLoop extends LoopBase {
    private static AutoManagerLoop instance;
    public static AutoManagerLoop getInstance(){if(instance == null){instance = new AutoManagerLoop();}return instance;}

    private final AutoManagerStatus status = AutoManagerStatus.getInstance();

    private final Timer autoTimer = new Timer();

    private AutoManagerLoop() {Subsystem = AutoManager.getInstance();}

    @Override
    protected void Enabled() {
        if(!DriverStation.isAutonomous()) {
            ArmTrajectory.setGlobalGrannyFactor(1.00); // teleop
            return;
        }
        ArmTrajectory.setGlobalGrannyFactor(1.00);  // auto
        if(status.EnabledState.IsInitState) {
            status.setCurrentAutoMode(status.getNewAutomode())
                  .setAutoRunning(true)
                  .setActionIndex(-1);
            SubsystemController.getInstance().loadConfiguration(status.getCurrentAutoMode().startConfiguration);
            autoTimer.start();
        }
        if(status.getActionIndex() < 0) {
            if(autoTimer.hasElapsed(status.getAutoConfiguration().initialDelay))
                status.setActionIndex(0);
        } else {
            checkAutoFinished();
            if(!status.getAutoRunning())
                return;
            Action action = status.getCurrentAutoMode().actionList.get(status.getActionIndex());
            action.onLoop();
            if(action.getEvaluatedDone()) {
                status.incrementActionIndex(1);
            }
            checkAutoFinished();
        }
    }

    private void checkAutoFinished() {
        if(status.getActionIndex() >= status.getCurrentAutoMode().actionList.size()) {
            status.setAutoRunning(false);
            System.out.println(status.getSelectedAutoMode().autoName + " finished");
        }
    }

    @Override
    protected void Disabled() {
        status.setSelectedAutoMode(status.getNT_SelectedAutoMode());
        status.setAutoConfiguration(new AutoConfiguration(
            status.getNT_StartingPose(),
            status.getNT_StartingPiece(),
            new GamePiece[]{
                status.getNT_StagedPiece0(),
                status.getNT_StagedPiece1(),
                status.getNT_StagedPiece2(),
                status.getNT_StagedPiece3()
            },
            status.getNT_InitialDelay()
        ));
    }
    @Override
    protected void Update() {
        
    }
}
