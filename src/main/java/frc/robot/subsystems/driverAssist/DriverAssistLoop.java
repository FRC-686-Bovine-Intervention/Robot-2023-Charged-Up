package frc.robot.subsystems.driverAssist;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.odometry.OdometryStatus;

public class DriverAssistLoop extends LoopBase {
    private static DriverAssistLoop instance;
    public static DriverAssistLoop getInstance(){if(instance == null){instance = new DriverAssistLoop();}return instance;}

    private final DriverAssistStatus status = DriverAssistStatus.getInstance();

    private DriverAssistLoop() {Subsystem = DriverAssist.getInstance();}

    private DriverAssistCommand prevCommand;
    
    @Override
    public void Update() {
        DriverAssistCommand newCommand = status.getCommand();
        if(!newCommand.equals(prevCommand))
        {
            ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
            waypoints.add(OdometryStatus.getInstance().getRobotPose());
            waypoints.add(OdometryStatus.getInstance().getRobotPose().plus(new Transform2d(new Translation2d(2,0), new Rotation2d())));
            waypoints.add(newCommand.getTargetPose());
            TrajectoryConfig trajectoryConfig = new TrajectoryConfig(1.0, 2.0);
            
            status.setTrajectory(TrajectoryGenerator.generateTrajectory(waypoints, trajectoryConfig));
        }

        prevCommand = newCommand;
    }

    @Override public void Enabled() {}
    @Override public void Disabled() {}
}
