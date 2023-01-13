package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.framework.LoopBase;

public class VisionLoop extends LoopBase {
    public static VisionLoop instance;
    public static VisionLoop getInstance(){
        if(instance == null){
            instance = new VisionLoop();
        }
        return instance;
    }

    private final PhotonCamera photonCamera;
    private final Transform3d cameraTransform;

    public final RobotPoseEstimator robotPoseEstimator;

    private final AprilTagFieldLayout aprilTagFieldLayout;


 
    private VisionLoop()
    {
        photonCamera = new PhotonCamera("TestCam");
        cameraTransform = new Transform3d();
    
        ArrayList<Pair<PhotonCamera, Transform3d>> cameraPairs = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        cameraPairs.add(new Pair<PhotonCamera, Transform3d>(photonCamera, cameraTransform));

        List<AprilTag> aprilTags = Arrays.asList(
            new AprilTag(1, new Pose3d()), 
            new AprilTag(2, new Pose3d(3.66, 0, 0, new Rotation3d()))
            );

        aprilTagFieldLayout = new AprilTagFieldLayout(aprilTags, 40, 20);

        robotPoseEstimator = new RobotPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.LOWEST_AMBIGUITY, 
            cameraPairs
            );
    
    }


    @Override
    public void Enabled() {
        
    }

    @Override
    public void Disabled() {
        
        
    }

    @Override
    public void Update() {
        // TODO Auto-generated method stub
        
    }
    
}
