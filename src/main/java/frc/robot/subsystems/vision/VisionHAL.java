package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.util.LimelightHelpers;

public class VisionHAL {
    private static VisionHAL instance;
    public static VisionHAL getInstance(){if(instance == null){instance = new VisionHAL();}return instance;}

    private static final String kLimelightName = "limelight";

    private static final String[] camNames = {
        // "Left Cam",
        "Right 2",
        "Back Cam"
    };
    public final PhotonCamera[] aprilTagCameras = new PhotonCamera[camNames.length];

    private VisionHAL()
    {
        if(RobotBase.isReal()) {
            for(int i = 0; i < aprilTagCameras.length; i++) {
                aprilTagCameras[i] = new PhotonCamera(camNames[i]);
            }
        } else {
        }
    }

    // AprilTags
    public PhotonPipelineResult[] getCameraResults() {
        PhotonPipelineResult[] results = new PhotonPipelineResult[aprilTagCameras.length];
        for (int i = 0; i < aprilTagCameras.length; i++) {
            results[i] = aprilTagCameras[i] != null ? aprilTagCameras[i].getLatestResult() : null;
        }
        return results;
    }

    // Limelight
    public boolean getTargetExists() {return LimelightHelpers.getTV(kLimelightName);}
    public double getTargetXAngle() {return LimelightHelpers.getTX(kLimelightName);}
    public double getTargetYAngle() {return LimelightHelpers.getTY(kLimelightName);}
    public double getCurrentArea() {return LimelightHelpers.getTA(kLimelightName);}
    public int getCurrentPipeline() {return (int)LimelightHelpers.getCurrentPipelineIndex(kLimelightName);}
    public VisionHAL setPipeline(int pipelineIndex) {LimelightHelpers.setPipelineIndex(kLimelightName, pipelineIndex); return this;}
}
