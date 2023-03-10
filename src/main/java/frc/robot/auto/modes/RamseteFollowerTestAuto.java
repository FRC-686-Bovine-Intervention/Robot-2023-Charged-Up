package frc.robot.auto.modes;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.auto.actions.IntakeCommandAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.RamseteFollowerAction;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class RamseteFollowerTestAuto extends AutoMode {
    public RamseteFollowerTestAuto()
    {
        RamseteController controller = new RamseteController(2, 0.7);

        TrajectoryConfig config = new TrajectoryConfig(1, 2);//new TrajectoryConfig(Units.feetToMeters(1), Units.feetToMeters(2));

        final double radius = 24;
        Pose2d origin = new Pose2d(Units.inchesToMeters(0),Units.inchesToMeters(0),Rotation2d.fromDegrees(0));
        Pose2d point1 = new Pose2d(Units.inchesToMeters(144-radius),Units.inchesToMeters(0),Rotation2d.fromDegrees(0));
        Pose2d point2 = new Pose2d(Units.inchesToMeters(144),Units.inchesToMeters(0+radius),Rotation2d.fromDegrees(90));
        Pose2d point3 = new Pose2d(Units.inchesToMeters(144),Units.inchesToMeters(114-radius),Rotation2d.fromDegrees(90));
        Pose2d point4 = new Pose2d(Units.inchesToMeters(144-radius),Units.inchesToMeters(114),Rotation2d.fromDegrees(180));
        Pose2d point5 = new Pose2d(Units.inchesToMeters(0+radius),Units.inchesToMeters(114),Rotation2d.fromDegrees(180));
        Pose2d point6 = new Pose2d(Units.inchesToMeters(0),Units.inchesToMeters(114-radius),Rotation2d.fromDegrees(270));
        Pose2d point7 = new Pose2d(Units.inchesToMeters(0),Units.inchesToMeters(0),Rotation2d.fromDegrees(270));
        // Pose2d point4 = origin;
        // final double distance = 5;
        // Pose2d origin = new Pose2d(distance * 0,0,Rotation2d.fromDegrees(0));
        // Pose2d point1 = new Pose2d(distance * 0.25,0.0,Rotation2d.fromDegrees(0));
        // Pose2d point2 = new Pose2d(distance * 0.5,0,Rotation2d.fromDegrees(0));
        // Pose2d point3 = new Pose2d(distance * 0.75,0,Rotation2d.fromDegrees(0));
        // Pose2d point4 = new Pose2d(distance * 1,0,Rotation2d.fromDegrees(0));

        ArrayList<Pose2d> path1 = new ArrayList<Pose2d>();
        // ArrayList<Pose2d> path2 = new ArrayList<Pose2d>();

        path1.add(origin);
        path1.add(point1);
        path1.add(point2);
        path1.add(point3);
        path1.add(point4);
        path1.add(point5);
        path1.add(point6);
        path1.add(point7);
        // path2.add(point2);
        // path2.add(point3);
        // path2.add(point4);

        Trajectory traj1 = TrajectoryGenerator.generateTrajectory(path1, config);
        // Trajectory traj2 = TrajectoryGenerator.generateTrajectory(path2, config);

        addAction(new ParallelAction(
            new RamseteFollowerAction(traj1, controller), 
            new IntakeCommandAction(new IntakeCommand(IntakeState.Grab))
        ));
        // addAction(new RamseteFollowerAction(traj2, controller));

        // addAction(new DriveStraightAction(0.25, 2));
    }
}
