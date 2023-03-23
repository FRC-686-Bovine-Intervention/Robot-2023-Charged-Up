package frc.robot.lib.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.DoubleStream;
import java.util.stream.LongStream;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionStatus;

public class AdvantageUtil {
    public static double[] deconstruct(Pose3d pose)
    {
        return new double[]{
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            pose.getRotation().getQuaternion().getW(),
            pose.getRotation().getQuaternion().getX(),
            pose.getRotation().getQuaternion().getY(),
            pose.getRotation().getQuaternion().getZ()
        };
    }
    public static double[] deconstructPose3ds(List<Pose3d> poses)
    {
        double[] r = new double[0];
        for (Pose3d pose : poses) {
            r = DoubleStream.concat(Arrays.stream(r), Arrays.stream(deconstruct(pose))).toArray();
        }
        return r;
    }
    public static double[] deconstruct(Pose2d pose)
    {
        return new double[]{
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        };
    }
    public static double[] deconstructPose2ds(List<Pose2d> poses)
    {
        double[] r = new double[0];
        for (Pose2d pose : poses) {
            r = DoubleStream.concat(Arrays.stream(r), Arrays.stream(deconstruct(pose))).toArray();
        }
        return r;
    }
    public static ArrayList<Pose3d> reconstructPose3d(double[] array)
    {
        ArrayList<Pose3d> r = new ArrayList<Pose3d>();
        for(int i = 0; i < Math.ceil(array.length/7); i++)
        {
            r.add(new Pose3d(new Translation3d(array[0+i],array[1+i],array[2+i]), 
                             new Rotation3d(
                             new Quaternion(array[3+i],array[4+i],array[5+i],array[6+i]))));
        }
        return r;
    }
    public static ArrayList<Pose2d> reconstructPose2d(double[] array)
    {
        ArrayList<Pose2d> r = new ArrayList<Pose2d>();
        for(int i = 0; i < Math.ceil(array.length/3); i++)
        {
            r.add(new Pose2d(new Translation2d(array[0+i],array[1+i]), 
                             new Rotation2d(array[2+i])));
        }
        return r;
    }

    public static void recordTrajectoryVector(Logger logger, String prefix, Matrix<N1,N3> matrix) {
        logger.recordOutput(prefix + "/position", matrix.get(0,0));
        logger.recordOutput(prefix + "/velocity", matrix.get(0,1));
        logger.recordOutput(prefix + "/acceleration", matrix.get(0,2));
    }

    public static void recordAprilTags(Logger logger, String prefix, List<AprilTag> aprilTags) {
        long[] tagIDs = new long[0];
        ArrayList<Pose3d> tagPoses = new ArrayList<Pose3d>();
        for(AprilTag tag : aprilTags) {
            tagIDs = LongStream.concat(Arrays.stream(tagIDs), Arrays.stream(new long[]{tag.ID})).toArray();
            tagPoses.add(tag.pose);
        }
        logger.recordOutput(prefix + "/IDs", tagIDs);
        logger.recordOutput(prefix + "/Poses", deconstructPose3ds(tagPoses));
    }

    public static void recordEstimatedRobotPoses(Logger logger, String prefix, List<EstimatedRobotPose> visionData) {
        ArrayList<AprilTag> aprilTags = new ArrayList<AprilTag>();
        ArrayList<Pose2d> visionPoses = new ArrayList<Pose2d>();

        for(EstimatedRobotPose data : visionData) {
            for (PhotonTrackedTarget target : data.targetsUsed) {
                aprilTags.add(new AprilTag(target.getFiducialId(), VisionStatus.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()));
            }
            visionPoses.add(data.estimatedPose.toPose2d());
        }

        recordAprilTags(logger, prefix + "/Visible Tags", aprilTags);
        logger.recordOutput(prefix + "/Vision Poses", deconstructPose2ds(visionPoses));
    }
}
