package frc.robot.subsystems.odometry;

import frc.robot.subsystems.framework.SubsystemBase;

public class Odometry extends SubsystemBase {
    private static Odometry instance;
    public static Odometry getInstance(){if(instance == null){instance = new Odometry();}return instance;}

    @Override
    public void init() {
        Loop = OdometryLoop.getInstance();
        Status = OdometryStatus.getInstance();
    }
}
