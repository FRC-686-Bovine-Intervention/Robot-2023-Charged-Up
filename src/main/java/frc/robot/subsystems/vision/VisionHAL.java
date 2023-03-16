package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.util.LimelightHelpers;

public class VisionHAL {
    private static VisionHAL instance;
    public static VisionHAL getInstance(){if(instance == null){instance = new VisionHAL();}return instance;}

    private static final String kLimelightName = "limelight";

    private static final String kLeftCamName = "Left Cam",
                                kRightCamName = "Right Cam";
    private final PhotonCamera cam1, cam2;

    private VisionHAL()
    {
        if(RobotBase.isReal()) {
            cam1 = new PhotonCamera(kLeftCamName);
            cam2 = null;//new PhotonCamera(kRightCamName);
        } else {
            cam1 = null;
            cam2 = null;
        }
    }

    // AprilTags
    public PhotonPipelineResult getCamera1Result() {return cam1 != null ? cam1.getLatestResult() : null;}
    public PhotonPipelineResult getCamera2Result() {return cam2 != null ? cam2.getLatestResult() : null;}

    // Limelight
    public boolean getTargetExists() {return LimelightHelpers.getTV(kLimelightName);}
    public double getTargetXAngle() {return LimelightHelpers.getTX(kLimelightName);}
    public double getTargetYAngle() {return LimelightHelpers.getTY(kLimelightName);}
    public double getCurrentArea() {return LimelightHelpers.getTA(kLimelightName);}
    public int getCurrentPipeline() {return (int)LimelightHelpers.getCurrentPipelineIndex(kLimelightName);}
    public VisionHAL setPipeline(int pipelineIndex) {LimelightHelpers.setPipelineIndex(kLimelightName, pipelineIndex); return this;}
}
