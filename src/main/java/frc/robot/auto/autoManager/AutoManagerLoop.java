package frc.robot.auto.autoManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.RamseteFollowerAction;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.framework.SubsystemController;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.odometry.OdometryCommand;

public class AutoManagerLoop extends LoopBase {
    private static AutoManagerLoop instance;
    public static AutoManagerLoop getInstance(){if(instance == null){instance = new AutoManagerLoop();}return instance;}

    private final AutoManagerStatus status = AutoManagerStatus.getInstance();

    private final Timer autoTimer = new Timer();

    private AutoManagerLoop() {Subsystem = AutoManager.getInstance();}

    @Override
    protected void Enabled() {
        if(!DriverStation.isAutonomous())
            return;
        if(status.EnabledState.IsInitState) {
            status.setCurrentAutoMode(status.getNewAutomode())
                  .setAutoRunning(true)
                  .setActionIndex(-1);
            SubsystemController.getInstance().loadConfiguration(status.getCurrentAutoMode().startConfiguration);
            autoTimer.start();
        }
        if(status.getActionIndex() < 0) {
            if(autoTimer.hasElapsed(status.getInitialDelay()))
                status.setActionIndex(0);
        } else {
            checkAutoFinished();
            if(!status.getAutoRunning())
                return;
            Action action = status.getCurrentAutoMode().actionList.get(status.getActionIndex());
            status.setActionTrajectory(action.getClass() == RamseteFollowerAction.class ? ((RamseteFollowerAction)action).controller.getTrajectory() : null);
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
        // if(status.getSelectedAutoMode() != status.getNT_SelectedAutoMode()) {
        //     status.setCurrentAutoMode(status.getNewAutomode());
        // }
        status.setInitialDelay(status.getNT_InitialDelay())
              .setSelectedAutoMode(status.getNT_SelectedAutoMode());
    }
    @Override
    protected void Update() {
        
    }
}
