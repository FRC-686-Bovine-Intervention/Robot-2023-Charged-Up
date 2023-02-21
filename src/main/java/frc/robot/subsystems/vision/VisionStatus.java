package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.LongStream;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.AdvantageUtil;
import frc.robot.subsystems.framework.StatusBase;

public class VisionStatus extends StatusBase {
    private static VisionStatus instance;
    public static VisionStatus getInstance(){if(instance == null){instance = new VisionStatus();}return instance;}

    private final Vision vision = Vision.getInstance();
    private final VisionHAL HAL = VisionHAL.getInstance();

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

    private ArrayList<Pose2d>   visionPoses = new ArrayList<Pose2d>();
    public ArrayList<Pose2d>    getVisionPoses()                                {return visionPoses;}
    public VisionStatus         setVisionPoses(ArrayList<Pose2d> visionPoses)   {this.visionPoses = visionPoses; return this;}

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

    private boolean         targetExists;
    protected boolean       getTargetExists()                       {return targetExists;}
    private VisionStatus    setTargetExists(boolean targetExists)   {this.targetExists = targetExists; return this;}

    private double          latestConeXAngle;
    public double           getLatestConeXAngle()                           {return latestConeXAngle;}
    protected VisionStatus  setLatestConeXAngle(double latestConeXAngle)    {this.latestConeXAngle = latestConeXAngle; return this;}

    private double          latestConeYAngle;
    public double           getLatestConeYAngle()                           {return latestConeYAngle;}
    protected VisionStatus  setLatestConeYAngle(double latestConeYAngle)    {this.latestConeYAngle = latestConeYAngle; return this;}

    private double          latestCubeXAngle;
    public double           getLatestCubeXAngle()                           {return latestCubeXAngle;}
    protected VisionStatus  setLatestCubeXAngle(double latestCubeXAngle)    {this.latestCubeXAngle = latestCubeXAngle; return this;}

    private double          latestCubeYAngle;
    public double           getLatestCubeYAngle()                           {return latestCubeYAngle;}
    protected VisionStatus  setLatestCubeYAngle(double latestCubeYAngle)    {this.latestCubeYAngle = latestCubeYAngle; return this;}

    @Override
    protected void updateInputs() {
        setCommand(vision.getVisionCommand());
        setVisionPoses(HAL.getVisionPoses());
        setCurrentPipeline(LimelightPipeline.getFromIndex(HAL.getCurrentPipeline()));
        setTargetXAngle(HAL.getTargetXAngle());
        setTargetYAngle(HAL.getTargetYAngle());
        setTargetExists(HAL.getTargetExists());
    }

    @Override
    protected void exportToTable(LogTable table) {
        table.put("Vision Poses", AdvantageUtil.deconstructPose2ds(visionPoses));
        long[] tagIDs = new long[0];
        ArrayList<Pose3d> tagPoses = new ArrayList<Pose3d>();

        for(AprilTag tag : HAL.getAprilTagFieldLayout().getTags())
        {
            tagPoses.add(tag.pose);
            tagIDs = LongStream.concat(Arrays.stream(tagIDs), Arrays.stream(new long[]{tag.ID})).toArray();
        }

        ArrayList<Pose3d> visiblePoses = new ArrayList<Pose3d>();

        for(AprilTag target : HAL.getVisibleTags())
            visiblePoses.add(target.pose);

        table.put("AprilTag Poses", AdvantageUtil.deconstructPose3ds(tagPoses));
        table.put("AprilTag IDs", tagIDs);
        table.put("Visible Tag Poses", AdvantageUtil.deconstructPose3ds(visiblePoses));

        table.put("Limelight/Current Pipeline", currentPipeline != null ? currentPipeline.name() : "null");
        table.put("Limelight/Target X Angle (Deg)", targetXAngle);
        table.put("Limelight/Target Y Angle (Deg)", targetYAngle);
        table.put("Limelight/Target Exists", targetExists);
    }

    @Override
    protected void importFromTable(LogTable table) {
        setVisionPoses(AdvantageUtil.reconstructPose2d(table.getDoubleArray("Vision Poses", null)));
        setCurrentPipeline(LimelightPipeline.getFromName(table.getString("Limelight/Current Pipeline", currentPipeline != null ? currentPipeline.name() : "null")));
        setTargetXAngle(table.getDouble("Limelight/Target X Angle (Deg)", targetXAngle));
        setTargetYAngle(table.getDouble("Limelight/Target Y Angle (Deg)", targetYAngle));
        setTargetExists(table.getBoolean("Limelight/Target Exists", targetExists));
    }

    @Override
    protected void processOutputs(Logger logger, String prefix) {
        HAL.setPipeline(getTargetPipeline().id);

        logger.recordOutput(prefix + "Limelight/Target Pipeline", getTargetPipeline().name());
        logger.recordOutput(prefix + "Limelight/Latest Cone X Angle (Deg)", getLatestConeXAngle());
        logger.recordOutput(prefix + "Limelight/Latest Cone Y Angle (Deg)", getLatestConeYAngle());
        logger.recordOutput(prefix + "Limelight/Latest Cube X Angle (Deg)", getLatestCubeXAngle());
        logger.recordOutput(prefix + "Limelight/Latest Cube Y Angle (Deg)", getLatestCubeYAngle());
    }

    @Override protected void processTable() {}
}
