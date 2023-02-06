package frc.robot.subsystems.odometry;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.robot.AdvantageUtil;
import frc.robot.subsystems.framework.StatusBase;

public class OdometryStatus extends StatusBase {
    private static OdometryStatus instance;
    public static OdometryStatus getInstance(){if(instance == null){instance = new OdometryStatus();}return instance;}

    private OdometryStatus()
    {
        Subsystem = Odometry.getInstance();
    }

    private Pose2d robotPose = new Pose2d();
    public Pose2d           getRobotPose()                  {return robotPose;}
    public OdometryStatus   setRobotPose(Pose2d robotPose)  {this.robotPose = robotPose; return this;}

    //TODO:change to meters per sec
    private WheelSpeeds robotSpeed = new WheelSpeeds();
    public WheelSpeeds      getRobotSpeedInPerSec()                         {return robotSpeed;}
    public OdometryStatus   setRobotSpeedInPerSec(WheelSpeeds robotSpeed)   {this.robotSpeed = robotSpeed; return this;}

    @Override
    public void recordOutputs(Logger logger, String prefix) {
        logger.recordOutput(prefix + "Robot Pose (Meters, Rad)", AdvantageUtil.deconstruct(getRobotPose()));
        logger.recordOutput(prefix + "Robot Speed (M|Sec)/Left", getRobotSpeedInPerSec().left);
        logger.recordOutput(prefix + "Robot Speed (M|Sec)/Right", getRobotSpeedInPerSec().right);

        logger.recordOutput(prefix + "Robot Forward Pose (Meters, Rad)", robotPose.plus(new Transform2d(new Translation2d(3,0),new Rotation2d())));
    }

    @Override public void exportToTable(LogTable table) {}
    @Override public void importFromTable(LogTable table) {}
    @Override public void updateInputs() {}
}
