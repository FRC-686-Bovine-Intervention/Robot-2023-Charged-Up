package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.framework.HALBase;

public class VisionHAL extends HALBase {
    public static VisionHAL instance;
    public static VisionHAL getInstance(){if(instance == null){instance = new VisionHAL();}return instance;}

    private final ArrayList<Pair<PhotonCamera, Transform3d>> cameraPairs = new ArrayList<Pair<PhotonCamera, Transform3d>>();

    private final AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(Arrays.asList(
        new AprilTag(21, new Pose3d())
        // new AprilTag(2, new Pose3d(3.66, 0, 0, new Rotation3d()))
        ), 40, 20);

    private final RobotPoseEstimator robotPoseEstimator;

    private VisionHAL()
    {
        cameraPairs.add(new Pair<PhotonCamera, Transform3d>(new PhotonCamera("TestCam"),new Transform3d()));
        
        robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameraPairs);
    }

    public Pose3d getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        if (result.isPresent()) {
            return result.get().getFirst();
        } else {
            return null;
        }
    }
}
