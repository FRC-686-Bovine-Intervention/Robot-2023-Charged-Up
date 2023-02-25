package frc.robot.subsystems.odometry;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.robot.AdvantageUtil;
import frc.robot.FieldDimensions;
import frc.robot.subsystems.framework.StatusBase;

public class OdometryStatus extends StatusBase {
    private static OdometryStatus instance;
    public static OdometryStatus getInstance(){if(instance == null){instance = new OdometryStatus();}return instance;}

    private OdometryStatus() {Subsystem = Odometry.getInstance();}

    private Pose2d          robotPose = new Pose2d();
    public Pose2d           getRobotPose()                  {return robotPose;}
    public OdometryStatus   setRobotPose(Pose2d robotPose)  {this.robotPose = robotPose; return this;}

    //TODO:change to meters per sec
    private WheelSpeeds     robotSpeed = new WheelSpeeds();
    public WheelSpeeds      getRobotSpeedInPerSec()                         {return robotSpeed;}
    public OdometryStatus   setRobotSpeedInPerSec(WheelSpeeds robotSpeed)   {this.robotSpeed = robotSpeed; return this;}

    @Override
    protected void processOutputs(Logger logger, String prefix) {
        logger.recordOutput(prefix + "Robot Pose (Meters, Rad)", AdvantageUtil.deconstruct(getRobotPose()));
        logger.recordOutput(prefix + "Robot Speed (M|Sec)/Left", getRobotSpeedInPerSec().left);
        logger.recordOutput(prefix + "Robot Speed (M|Sec)/Right", getRobotSpeedInPerSec().right);

        logger.recordOutput(prefix + "Pose Bounding Boxes/In Community", FieldDimensions.community.withinBounds(robotPose));
        logger.recordOutput(prefix + "Pose Bounding Boxes/In Community without Charge Station", FieldDimensions.communityWithoutChargeStation.withinBounds(robotPose));
        logger.recordOutput(prefix + "Pose Bounding Boxes/In Charge Station", FieldDimensions.chargeStation.withinBounds(robotPose));
    }
    
    @Override protected void updateInputs() {}
    @Override protected void exportToTable(LogTable table) {}
    @Override protected void importFromTable(LogTable table) {}
    @Override protected void processTable() {}
}
