package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionHAL {
    private static VisionHAL instance;
    public static VisionHAL getInstance(){if(instance == null){instance = new VisionHAL();}return instance;}

    private final ArrayList<Pair<PhotonCamera, Transform3d>> cameraPairs = new ArrayList<Pair<PhotonCamera, Transform3d>>();

    private final ArrayList<AprilTag> aprilTags = new ArrayList<AprilTag>();
    private final AprilTagFieldLayout aprilTagFieldLayout;
    public AprilTagFieldLayout getAprilTagFieldLayout() {return aprilTagFieldLayout;}

    // private final RobotPoseEstimator robotPoseEstimator;

    private VisionHAL()
    {
        aprilTags.add(new AprilTag(1, new Pose3d(Units.inchesToMeters(162), Units.inchesToMeters(-59), Units.inchesToMeters(78), new Rotation3d(0,0,Math.PI))));
        aprilTags.add(new AprilTag(2, new Pose3d(Units.inchesToMeters(162), Units.inchesToMeters(54), Units.inchesToMeters(79), new Rotation3d(0,0,Math.PI))));
        aprilTagFieldLayout = new AprilTagFieldLayout(aprilTags, Units.inchesToMeters(40*12), Units.inchesToMeters(20*12));

        if(RobotBase.isReal())
        {
            cameraPairs.add(new Pair<PhotonCamera, Transform3d>(
                            new PhotonCamera("TestCam"),
                            new Transform3d(new Translation3d(Units.inchesToMeters(15), 0, Units.inchesToMeters(30)), new Rotation3d())));
        }
        
        // robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameraPairs);
    }

    public ArrayList<AprilTag> getVisibleTags()
    {
        ArrayList<AprilTag> list = new ArrayList<AprilTag>();

        for(Pair<PhotonCamera, Transform3d> cameraPair : cameraPairs) {
            PhotonCamera camera = cameraPair.getFirst();
            List<PhotonTrackedTarget> targets = camera.getLatestResult().targets;

            for(PhotonTrackedTarget target : targets)
            {
                Optional<Pose3d> fiducialPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (fiducialPose.isEmpty()) {continue;}
                
                list.add(new AprilTag(target.getFiducialId(), fiducialPose.get()));
            }
        }

        return list;
    }

    public ArrayList<Pose2d> getVisionPoses()
    {
        ArrayList<Pose2d> list = new ArrayList<Pose2d>();

        for(Pair<PhotonCamera, Transform3d> cameraPair : cameraPairs) {
            PhotonCamera camera = cameraPair.getFirst();
            Transform3d cameraTransform = cameraPair.getSecond();
            List<PhotonTrackedTarget> targets = camera.getLatestResult().targets;

            for(PhotonTrackedTarget target : targets)
            {
                Optional<Pose3d> fiducialPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (fiducialPose.isEmpty()) {continue;}

                Pose3d targetPose = fiducialPose.get();
                Pose3d botPose = targetPose
                                    .transformBy(target.getBestCameraToTarget().inverse())
                                    .transformBy(cameraTransform.inverse());
                
                list.add(botPose.toPose2d());
            }
        }

        return list;
    }

    // public Pose3d getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose) {
    //     robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    
    //     Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    //     if (result.isPresent()) {
    //         return result.get().getFirst();
    //     } else {
    //         return null;
    //     }
    // }
}
