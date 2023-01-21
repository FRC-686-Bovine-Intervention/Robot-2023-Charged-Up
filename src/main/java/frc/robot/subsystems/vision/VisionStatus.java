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

    private VisionStatus()
    {
        Subsystem = Vision.getInstance();
    }

    private final VisionHAL HAL = VisionHAL.getInstance();

    //@Log(name = "Vision Pose")
    private ArrayList<Pose2d> visionPoses = new ArrayList<Pose2d>();
    public ArrayList<Pose2d>    getVisionPoses()                                {return visionPoses;}
    public VisionStatus         setVisionPoses(ArrayList<Pose2d> visionPoses)   {this.visionPoses = visionPoses; return this;}

    @Override
    public void exportToTable(LogTable table) {
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
        {
            visiblePoses.add(target.pose);
        }

        table.put("AprilTag Poses", AdvantageUtil.deconstructPose3ds(tagPoses));
        table.put("AprilTag IDs", tagIDs);
        table.put("Visible Tag Poses", AdvantageUtil.deconstructPose3ds(visiblePoses));
    }

    @Override
    public void importFromTable(LogTable table) {
        setVisionPoses(AdvantageUtil.reconstructPose2d(table.getDoubleArray("Vision Poses", null)));
    }

    @Override
    public void updateInputs() {
        setVisionPoses(HAL.getVisionPoses());
    }

    @Override public void recordOutputs(Logger logger, String prefix) {}
}
