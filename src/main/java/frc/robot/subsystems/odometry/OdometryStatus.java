package frc.robot.subsystems.odometry;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.robot.AdvantageUtil;
import frc.robot.subsystems.framework.StatusBase;

public class OdometryStatus extends StatusBase {
    private static OdometryStatus instance;
    public static OdometryStatus getInstance(){if(instance == null){instance = new OdometryStatus();}return instance;}

    private Pose2d robotPose = new Pose2d();
    public Pose2d getRobotPose()                            {return robotPose;}
    public OdometryStatus setRobotPose(Pose2d robotPose)    {this.robotPose = robotPose; return this;}

    private WheelSpeeds robotSpeed = new WheelSpeeds();
    public WheelSpeeds getRobotSpeed()                          {return robotSpeed;}
    public OdometryStatus setRobotSpeed(WheelSpeeds robotSpeed) {this.robotSpeed = robotSpeed; return this;}

    @Override
    public void recordOutputs(String prefix) {
        Logger.getInstance().recordOutput(prefix + "/Robot Pose (Meters, Rad)", AdvantageUtil.deconstruct(getRobotPose()));
        Logger.getInstance().recordOutput(prefix + "/Robot Speed (M|Sec)/Left", getRobotSpeed().left);
        Logger.getInstance().recordOutput(prefix + "/Robot Speed (M|Sec)/Right", getRobotSpeed().right);
    }

    @Override public void exportToTable(LogTable table, String prefix) {}
    @Override public void importFromTable(LogTable table, String prefix) {}
    @Override public void updateInputs() {}
}
