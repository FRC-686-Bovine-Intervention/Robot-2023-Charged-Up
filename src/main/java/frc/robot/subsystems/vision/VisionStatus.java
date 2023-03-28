package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import frc.robot.FieldDimensions;
import frc.robot.RobotConfiguration;
import frc.robot.lib.util.AdvantageUtil;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.framework.StatusBase;

public class VisionStatus extends StatusBase {
    private static VisionStatus instance;
    public static VisionStatus getInstance(){if(instance == null){instance = new VisionStatus();}return instance;}

    private final Vision vision = Vision.getInstance();
    private final VisionHAL HAL = VisionHAL.getInstance();
    private final ArmStatus armStatus = ArmStatus.getInstance();

    public enum LimelightPipeline {
        Pole(2),
        Cone(0),
        Cube(1);
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

    public static final AprilTagFieldLayout aprilTagFieldLayout;
    static {
        try {
            aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            throw new NullPointerException("Failed to load AprilTagFieldLayout");
        }
    }

    private VisionStatus() {
        Subsystem = vision;
        // for(int i = 0; i < aprilTagCameras.length; i++) {
        //     aprilTagCameras[i] = new PhotonHalHelper(HAL.aprilTagCameras[i], turretToCameras[i], PoseStrategy.MULTI_TAG_PNP, aprilTagFieldLayout);
        // }
    }

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
    public double        getTargetXAngle()                       {return targetXAngle;}
    private VisionStatus    setTargetXAngle(double targetXAngle)    {this.targetXAngle = targetXAngle; return this;}

    private double          targetYAngle;
    public double        getTargetYAngle()                       {return targetYAngle;}
    private VisionStatus    setTargetYAngle(double targetYAngle)    {this.targetYAngle = targetYAngle; return this;}

    private double          currentArea;
    public double        getCurrentArea()                    {return currentArea;}
    private VisionStatus    setCurrentArea(double currentArea)  {this.currentArea = currentArea; return this;}

    private boolean         targetExists;
    public boolean       getTargetExists()                       {return targetExists;}
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

    private boolean         coneExists;
    public boolean          getConeExists()                             {return coneExists;}
    protected VisionStatus  setConeExists(Boolean coneExists)        {this.coneExists = coneExists; return this;}


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
        private static final Translation3d camOrigin = new Translation3d();
        private static final double kMaxDistToTarget = FieldDimensions.Community.chargingStationOuterX;
        
        public Pose2d getRobotPose() {
            return aprilTag.pose.transformBy(camToTarget.inverse()).transformBy(robotToCam.inverse()).toPose2d();
        }
        public double getDistanceToTarget() {
            return camOrigin.getDistance(camToTarget.getTranslation());
        }
        public boolean isGoodData() {
            return getDistanceToTarget() < kMaxDistToTarget;
        }
        public Matrix<N3, N1> getStdDevs() {
            double xyCoefficient = 0.1;
            double thetaCoefficient = 0.1;
            double xyBias = 0.1;
            double thetaBias = 0.1;
            
            double distance = getDistanceToTarget();
            double xyStdDev = xyCoefficient * distance * distance + xyBias;
            double thetaStdDev = thetaCoefficient * distance * distance + thetaBias;

            return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(xyStdDev, xyStdDev, thetaStdDev);
        }
        public static void record(Logger logger, String prefix, List<VisionData> visionData) {
            // ArrayList<AprilTag> aprilTags = new ArrayList<AprilTag>();
            ArrayList<Pose2d> visionPoses = new ArrayList<Pose2d>();

            for(VisionData data : visionData) {
                // for (PhotonTrackedTarget target : data.targetsUsed) {
                //     aprilTags.add(new AprilTag(target.getFiducialId(), VisionStatus.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()));
                // }
                visionPoses.add(data.getRobotPose());
            }

            // recordAprilTags(logger, prefix + "/Visible Tags", aprilTags);
            logger.recordOutput(prefix + "/Vision Poses", AdvantageUtil.deconstructPose2ds(visionPoses));
        }
    }
    private ArrayList<VisionData>   visionData = new ArrayList<VisionData>();
    public ArrayList<VisionData>    getVisionData()                                         {return visionData;}
    protected VisionStatus                  setVisionData(ArrayList<VisionData> visionData) {this.visionData = visionData; return this;}

    public static final Transform3d[] turretToCameras = {
        // Right Cam
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.212),
                Units.inchesToMeters(-7.639),
                Units.inchesToMeters(23.375)
            ),
            new Rotation3d(
                0,
                0,
                Units.degreesToRadians(+30.0)
            )
        ),
        // Left Cam
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.212),
                Units.inchesToMeters(+7.639),
                Units.inchesToMeters(23.375)
            ),
            new Rotation3d(
                0,
                0,
                Units.degreesToRadians(-30.0)
            )
        ),
        // Back Cam
        // new Transform3d(
        //     new Translation3d(
        //         Units.inchesToMeters(-4),
        //         Units.inchesToMeters(+11.25/20),
        //         Units.inchesToMeters(+24.75)
        //     ),
        //     new Rotation3d(
        //         0,
        //         0,
        //         Units.degreesToRadians(+180)
        //     )
        // )
    };

    private final PhotonPipelineResult[]    camResults = new PhotonPipelineResult[HAL.aprilTagCameras.length];
    public PhotonPipelineResult[]           getCamResults() {return camResults;}

    // private final PhotonHalHelper[] aprilTagCameras = new PhotonHalHelper[HAL.aprilTagCameras.length];
    // public PhotonHalHelper[]        getAprilTagCameras()        {return aprilTagCameras;}
    // public PhotonHalHelper          getAprilTagCamera(int i)    {return aprilTagCameras[i];}

    @Override
    protected void updateInputs() {
        setCommand(vision.getVisionCommand());

        // for(int i = 0; i < aprilTagCameras.length; i++) {
        //     aprilTagCameras[i].setCameraTransform(armStatus.getRobotToTurret().plus(turretToCameras[i]))
        //                       .updateInputs();
        // }
        for (int i = 0; i < camResults.length; i++) {
            camResults[i] = HAL.getCameraResults()[i];
        }
        
        setCurrentPipeline(LimelightPipeline.getFromIndex(HAL.getCurrentPipeline()));
        setTargetXAngle(HAL.getTargetXAngle());
        setTargetYAngle(HAL.getTargetYAngle());
        setCurrentArea(HAL.getCurrentArea());
        setTargetExists(HAL.getTargetExists());
    }

    @Override
    protected void exportToTable(LogTable table) {
        // for(PhotonHalHelper cam : aprilTagCameras) {
        //     cam.exportToTable(table);
        // }

        table.put("Limelight/Current Pipeline", currentPipeline != null ? currentPipeline.name() : "null");

        table.put("Limelight/Target X Angle (Deg)", targetXAngle);
        table.put("Limelight/Target Y Angle (Deg)", targetYAngle);
        table.put("Limelight/Current Area (Deg)", currentArea);
        table.put("Limelight/Target Exists", targetExists);
    }

    @Override
    protected void importFromTable(LogTable table) {
        // for(PhotonHalHelper cam : aprilTagCameras) {
        //     cam.importFromTable(table);
        // }

        setCurrentPipeline(LimelightPipeline.getFromName(table.getString("Limelight/Current Pipeline", currentPipeline != null ? currentPipeline.name() : "null")));
        setTargetXAngle(table.getDouble("Limelight/Target X Angle (Deg)", targetXAngle));
        setTargetYAngle(table.getDouble("Limelight/Target Y Angle (Deg)", targetYAngle));
        setTargetYAngle(table.getDouble("Limelight/Current Area (Deg)", targetYAngle));
        setTargetExists(table.getBoolean("Limelight/Target Exists", targetExists));
    }

    @Override
    protected void processTable() {
        // for(PhotonHalHelper cam : aprilTagCameras) {
        //     cam.processTable();
        // }
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
        // AdvantageUtil.recordEstimatedRobotPoses(logger, prefix + "AprilTags", visionData);
        VisionData.record(logger, prefix + "AprilTags", visionData);
    }

    
    @Override protected void loadConfiguration(RobotConfiguration configuration) {}
}
