package frc.robot.auto.autoManager;

import frc.robot.auto.modes.AutoTrajectories;
import frc.robot.subsystems.framework.SubsystemBase;

public class AutoManager extends SubsystemBase {
    private static AutoManager instance;
    public static AutoManager getInstance(){if(instance == null){instance = new AutoManager();}return instance;}

    private AutoManager() {
        AutoTrajectories.loadTrajectories();
    }

    @Override
    protected void init() {
        Loop = AutoManagerLoop.getInstance();
        Status = AutoManagerStatus.getInstance();
    }
}
