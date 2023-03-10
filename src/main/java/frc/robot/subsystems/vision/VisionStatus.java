package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.AdvantageUtil;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.framework.StatusBase;
import frc.robot.subsystems.odometry.OdometryStatus;

public class VisionStatus extends StatusBase {
    private static VisionStatus instance;
    public static VisionStatus getInstance(){if(instance == null){instance = new VisionStatus();}return instance;}

    private final Vision vision = Vision.getInstance();
    private final VisionHAL HAL = VisionHAL.getInstance();
    private final ArmStatus armStatus = ArmStatus.getInstance();

    public enum LimelightPipeline {
        Pole(0),
        Cone(1),
        Cube(2);
        public final int id;
        LimelightPipeline(int id){this.id = id;}
        public static LimelightPipeline getFromName(String name)
        {
            LimelightPipeline r = null;
            for(LimelightPipeline pipeline : LimelightPipeline.values())
            {
                if(pipeline.name().equals(name))
                {
                    r = pipeline;
                    break;
                }
            }
            return r;
        }
        public static LimelightPipeline getFromIndex(int index)
        {
            LimelightPipeline r = null;
            for(LimelightPipeline pipeline : LimelightPipeline.values())
            {
                if(pipeline.id == index)
                {
                    r = pipeline;
                    break;
                }
            }
            return r;
        }
    }

    private VisionStatus() {Subsystem = vision;}

    private VisionCommand   command = new VisionCommand();
    protected VisionCommand getCommand()                        {return command;}
    private VisionStatus    setCommand(VisionCommand command)   {this.command = command; return this;}

    // Limelight
    private LimelightPipeline   currentPipeline = null;
    public LimelightPipeline    getCurrentPipeline()                                    {return currentPipeline;}
    private VisionStatus        setCurrentPipeline(LimelightPipeline currentPipeline)   {this.currentPipeline = currentPipeline; return this;}

    private LimelightPipeline   targetPipeline = LimelightPipeline.Cone;
    public LimelightPipeline    getTargetPipeline()                                 {return targetPipeline;}
    public VisionStatus         setTargetPipeline(LimelightPipeline targetPipeline) {this.targetPipeline = targetPipeline; return this;}

    private double          targetXAngle;
    protected double        getTargetXAngle()                       {return targetXAngle;}
    private VisionStatus    setTargetXAngle(double targetXAngle)    {this.targetXAngle = targetXAngle; return this;}

    private double          targetYAngle;
    protected double        getTargetYAngle()                       {return targetYAngle;}
    private VisionStatus    setTargetYAngle(double targetYAngle)    {this.targetYAngle = targetYAngle; return this;}

    private double          currentArea;
    protected double        getCurrentArea()                    {return currentArea;}
    private VisionStatus    setCurrentArea(double currentArea)  {this.currentArea = currentArea; return this;}

    private boolean         targetExists;
    protected boolean       getTargetExists()                       {return targetExists;}
    private VisionStatus    setTargetExists(boolean targetExists)   {this.targetExists = targetExists; return this;}


    private double          latestConeXAngle;
    public double           getLatestConeXAngle()                           {return latestConeXAngle;}
    protected VisionStatus  setLatestConeXAngle(double latestConeXAngle)    {this.latestConeXAngle = latestConeXAngle; return this;}

    private double          latestConeYAngle;
    public double           getLatestConeYAngle()                           {return latestConeYAngle;}
    protected VisionStatus  setLatestConeYAngle(double latestConeYAngle)    {this.latestConeYAngle = latestConeYAngle; return this;}

    private double          latestConeArea;
    public double           getLatestConeArea()                             {return latestConeArea;}
    protected VisionStatus  setLatestConeArea(double latestConeArea)        {this.latestConeArea = latestConeArea; return this;}

    private boolean          coneExists;
    public boolean           getConeExists()                             {return coneExists;}
    protected VisionStatus   setConeExists(Boolean coneExists)        {this.coneExists = coneExists; return this;}


    private double          latestCubeXAngle;
    public double           getLatestCubeXAngle()                           {return latestCubeXAngle;}
    protected VisionStatus  setLatestCubeXAngle(double latestCubeXAngle)    {this.latestCubeXAngle = latestCubeXAngle; return this;}

    private double          latestCubeYAngle;
    public double           getLatestCubeYAngle()                           {return latestCubeYAngle;}
    protected VisionStatus  setLatestCubeYAngle(double latestCubeYAngle)    {this.latestCubeYAngle = latestCubeYAngle; return this;}

    private double          latestCubeArea;
    public double           getLatestCubeArea()                             {return latestCubeArea;}
    protected VisionStatus  setLatestCubeArea(double latestCubeArea)        {this.latestCubeArea = latestCubeArea; return this;}

    private boolean          cubeExists;
    public boolean           getCubeExists()                            {return cubeExists;}
    protected VisionStatus   setCubeExists(Boolean cubeExists)          {this.cubeExists = cubeExists; return this;}

    // AprilTags
    public record VisionData(AprilTag aprilTag, Transform3d camToTarget, Transform3d robotToCam, double timestamp) {
        public Pose2d getRobotPose() {
            return aprilTag.pose.transformBy(camToTarget.inverse()).transformBy(robotToCam.inverse()).toPose2d();
        }
        public Pose3d getReprojection() {
            return new Pose3d(OdometryStatus.getInstance().getRobotPose()).transformBy(robotToCam).transformBy(camToTarget);
        }
        public double getReprojectionError() {
            return aprilTag.pose.getTranslation().getDistance(getReprojection().getTranslation());
        }
        public double getKalmanError() {
            return OdometryStatus.getInstance().getRobotPose().getTranslation().getDistance(getRobotPose().getTranslation());
        }
        public Matrix<N3, N1> getStdDevs() {
            double xyCoefficient = 0.1;
            double thetaCoefficient = 0.1;
            double xyBias = 0.1;
            double thetaBias = 0.1;
            
            double distance = getKalmanError();
            double xyStdDev = xyCoefficient * distance * distance + xyBias;
            double thetaStdDev = thetaCoefficient * distance * distance + thetaBias;

            return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(xyStdDev, xyStdDev, thetaStdDev);
        }
    }
    private ArrayList<VisionData>   visionData = new ArrayList<VisionData>();
    public ArrayList<VisionData>    getVisionData()                                 {return visionData;}
    protected VisionStatus          setVisionData(ArrayList<VisionData> visionData) {this.visionData = visionData; return this;}
    
    // Camera 1
    private Transform3d turretToCamera1 = 
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11.33),
                Units.inchesToMeters(+5.806),
                Units.inchesToMeters(20.375)
            ),
            new Rotation3d(
                0,
                0,
                Units.degreesToRadians(-20.0)
            )
        ); //TODO
    public Transform3d  getTurretToCamera1()    {return turretToCamera1;};
    public Transform3d  getRobotToCamera1()     {return armStatus.getRobotToTurret().plus(turretToCamera1);}

    private PhotonPipelineResult    camera1Result;
    public PhotonPipelineResult     getCamera1Result()                                      {return camera1Result;}
    private VisionStatus            setCamera1Result(PhotonPipelineResult camera1Result)    {this.camera1Result = camera1Result; return this;}
    
    // Camera 2
    private Transform3d turretToCamera2 = 
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11.33),
                Units.inchesToMeters(-5.806),
                Units.inchesToMeters(20.375)
            ),
            new Rotation3d(
                0,
                0,
                Units.degreesToRadians(+20.0)
            )
        ); //TODO
    public Transform3d  getTurretToCamera2()    {return turretToCamera2;}
    public Transform3d  getRobotToCamera2()     {return armStatus.getRobotToTurret().plus(turretToCamera2);}

    private PhotonPipelineResult    camera2Result;
    public PhotonPipelineResult     getCamera2Result()                                      {return camera2Result;}
    private VisionStatus            setCamera2Result(PhotonPipelineResult camera2Result)    {this.camera2Result = camera2Result; return this;}

    @Override
    protected void updateInputs() {
        setCommand(vision.getVisionCommand());

        setCamera1Result(HAL.getCamera1Result());
        setCamera2Result(HAL.getCamera2Result());

        setCurrentPipeline(LimelightPipeline.getFromIndex(HAL.getCurrentPipeline()));
        setTargetXAngle(HAL.getTargetXAngle());
        setTargetYAngle(HAL.getTargetYAngle());
        setCurrentArea(HAL.getCurrentArea());
        setTargetExists(HAL.getTargetExists());
    }

    @Override
    protected void exportToTable(LogTable table) {
        if (camera1Result != null) {
            Packet camPacket = new Packet(camera1Result.getPacketSize());
            camPacket = camera1Result.populatePacket(camPacket);
            table.put("AprilTags/Camera 1 Results", camPacket.getData());
        }

        if (camera2Result != null) {
            Packet camPacket = new Packet(camera2Result.getPacketSize());
            camPacket = camera2Result.populatePacket(camPacket);
            table.put("AprilTags/Camera 2 Results", camPacket.getData());
        }

        table.put("Limelight/Current Pipeline", currentPipeline != null ? currentPipeline.name() : "null");

        table.put("Limelight/Target X Angle (Deg)", targetXAngle);
        table.put("Limelight/Target Y Angle (Deg)", targetYAngle);
        table.put("Limelight/Current Area (Deg)", currentArea);
        table.put("Limelight/Target Exists", targetExists);
    }

    @Override
    protected void importFromTable(LogTable table) {
        camera1Result.createFromPacket(new Packet(table.getRaw("AprilTags/Camera 1 Results", null)));
        camera2Result.createFromPacket(new Packet(table.getRaw("AprilTags/Camera 2 Results", null)));

        setCurrentPipeline(LimelightPipeline.getFromName(table.getString("Limelight/Current Pipeline", currentPipeline != null ? currentPipeline.name() : "null")));
        setTargetXAngle(table.getDouble("Limelight/Target X Angle (Deg)", targetXAngle));
        setTargetYAngle(table.getDouble("Limelight/Target Y Angle (Deg)", targetYAngle));
        setTargetYAngle(table.getDouble("Limelight/Current Area (Deg)", targetYAngle));
        setTargetExists(table.getBoolean("Limelight/Target Exists", targetExists));
    }

    @Override
    protected void processOutputs(Logger logger, String prefix) {
        HAL.setPipeline(getTargetPipeline().id);

        // Limelight
        logger.recordOutput(prefix + "Limelight/Target Pipeline", getTargetPipeline().name());
        logger.recordOutput(prefix + "Limelight/Latest Cone X Angle (Deg)", getLatestConeXAngle());
        logger.recordOutput(prefix + "Limelight/Latest Cone Y Angle (Deg)", getLatestConeYAngle());
        logger.recordOutput(prefix + "Limelight/Latest Cone Area (Deg)",    getLatestConeArea());
        logger.recordOutput(prefix + "Limelight/Latest Cube X Angle (Deg)", getLatestCubeXAngle());
        logger.recordOutput(prefix + "Limelight/Latest Cube Y Angle (Deg)", getLatestCubeYAngle());
        logger.recordOutput(prefix + "Limelight/Latest Cube Area (Deg)",    getLatestCubeArea());
        
        // AprilTags
        // logger.recordOutput(prefix + "AprilTags/Robot to Camera/1", new Pose3d().transformBy(getRobotToCamera1()));
        // logger.recordOutput(prefix + "AprilTags/Robot to Camera/2", new Pose3d().transformBy(getRobotToCamera2()));
        logger.recordOutput(prefix + "AprilTags/Field to Camera", new Pose3d[]{new Pose3d(OdometryStatus.getInstance().getRobotPose()).transformBy(getRobotToCamera1()), new Pose3d(OdometryStatus.getInstance().getRobotPose()).transformBy(getRobotToCamera2())});
        AdvantageUtil.recordVisionData(logger, prefix + "AprilTags", visionData);
    }

    @Override protected void processTable() {}
}
