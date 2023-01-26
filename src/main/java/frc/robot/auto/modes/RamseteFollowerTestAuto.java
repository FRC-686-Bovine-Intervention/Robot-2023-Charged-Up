package frc.robot.auto.modes;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.auto.actions.RamseteFollowerAction;

public class RamseteFollowerTestAuto extends AutoMode {
    public RamseteFollowerTestAuto()
    {
        RamseteController controller = new RamseteController(1,1);

        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(1), Units.feetToMeters(2));

        Pose2d origin = new Pose2d(0,0,Rotation2d.fromDegrees(0));
        Pose2d point1 = new Pose2d(0.5,0.5,Rotation2d.fromDegrees(90));
        Pose2d point2 = new Pose2d(0,1,Rotation2d.fromDegrees(180));
        Pose2d point3 = new Pose2d(-0.5,0.5,Rotation2d.fromDegrees(270));

        ArrayList<Pose2d> path1 = new ArrayList<Pose2d>();
        ArrayList<Pose2d> path2 = new ArrayList<Pose2d>();

        path1.add(origin);
        path1.add(point1);
        path1.add(point2);
        path2.add(point2);
        path2.add(point3);
        path2.add(origin);

        Trajectory traj1 = TrajectoryGenerator.generateTrajectory(path1, config);
        Trajectory traj2 = TrajectoryGenerator.generateTrajectory(path2, config);

        addAction(new RamseteFollowerAction(traj1, controller));
        addAction(new RamseteFollowerAction(traj2, controller));

        // addAction(new DriveStraightAction(0.25, 2));
    }
}
