package frc.robot.subsystems.odometry;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.robot.RobotConfiguration;
import frc.robot.lib.util.AdvantageUtil;
import frc.robot.subsystems.framework.StatusBase;

public class OdometryStatus extends StatusBase {
    private static OdometryStatus instance;
    public static OdometryStatus getInstance(){if(instance == null){instance = new OdometryStatus();}return instance;}

    private final Odometry odometry = Odometry.getInstance();

    private OdometryStatus() {Subsystem = odometry;}

    private OdometryCommand command;
    public OdometryCommand  getCommand()                        {return command;}
    private OdometryStatus  setCommand(OdometryCommand command) {this.command = command; return this;}

    private Pose2d          robotPose = new Pose2d();
    public Pose2d           getRobotPose()                  {return robotPose;}
    public OdometryStatus   setRobotPose(Pose2d robotPose)  {this.robotPose = robotPose; return this;}

    private boolean         ignoreVision;
    public boolean          getIgnoreVision()                       {return ignoreVision;}
    public OdometryStatus   setIgnoreVision(boolean ignoreVision)   {this.ignoreVision = ignoreVision; return this;}

    //TODO:change to meters per sec
    private WheelSpeeds     robotSpeed = new WheelSpeeds();
    public WheelSpeeds      getRobotSpeedInPerSec()                         {return robotSpeed;}
    public WheelSpeeds      getRobotSpeedMeterPerSec()                      {return new WheelSpeeds(Units.inchesToMeters(robotSpeed.left), Units.inchesToMeters(robotSpeed.right));}
    public OdometryStatus   setRobotSpeedInPerSec(WheelSpeeds robotSpeed)   {this.robotSpeed = robotSpeed; return this;}

    @Override
    protected void updateInputs() {
        setCommand(odometry.getCommand());
    }

    @Override
    protected void processOutputs(Logger logger, String prefix) {
        logger.recordOutput(prefix + "Robot Pose (Meters, Rad)", AdvantageUtil.deconstruct(getRobotPose()));
        logger.recordOutput(prefix + "Is Ignoring Vision", getIgnoreVision());
        // logger.recordOutput(prefix + "Robot Speed (M|Sec)/Left", getRobotSpeedMeterPerSec().left);
        // logger.recordOutput(prefix + "Robot Speed (M|Sec)/Right", getRobotSpeedMeterPerSec().right);

        // logger.recordOutput(prefix + "Pose Bounding Boxes/In Community", FieldDimensions.community.withinBounds(AllianceFlipUtil.apply(robotPose)));
        // logger.recordOutput(prefix + "Pose Bounding Boxes/In Community without Charge Station", FieldDimensions.communityWithoutChargeStation.withinBounds(AllianceFlipUtil.apply(robotPose)));
        // logger.recordOutput(prefix + "Pose Bounding Boxes/In Charge Station", FieldDimensions.chargeStation.withinBounds(AllianceFlipUtil.apply(robotPose)));

        odometry.setCommand(new OdometryCommand());
    }

    @Override
    protected void loadConfiguration(RobotConfiguration configuration) {
        odometry.getCommand().setResetPose(configuration.robotPose);
    }
    
    @Override protected void exportToTable(LogTable table) {}
    @Override protected void importFromTable(LogTable table) {}
    @Override protected void processTable() {}
}
