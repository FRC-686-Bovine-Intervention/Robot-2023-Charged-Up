package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.vision.VisionStatus.LimelightPipeline;
import frc.robot.subsystems.vision.VisionStatus.VisionData;

public class VisionLoop extends LoopBase {
    private static VisionLoop instance;
    public static VisionLoop getInstance(){if(instance == null){instance = new VisionLoop();}return instance;}

    private final VisionStatus status = VisionStatus.getInstance();
    private final ArmStatus armStatus = ArmStatus.getInstance();

    private static final double kPipelineToggleDelay = 2;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    private VisionLoop() {
        Subsystem = Vision.getInstance();
        
        try {
            aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            throw new NullPointerException("Failed to load AprilTagFieldLayout");
        }
    }

    private double pipelineStartTimestamp;
    
    @Override
    public void Update() {
        VisionCommand newCommand = status.getCommand();
        double currentTime = Timer.getFPGATimestamp();

        // AprilTags
        // Camera 1
        ArrayList<VisionData> camera1Data = new ArrayList<VisionData>();
        if (status.getCamera1Result() != null) {
            for(PhotonTrackedTarget target : status.getCamera1Result().targets) {
                Optional<Pose3d> fiducialPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (fiducialPose.isEmpty()) {continue;}
                
                camera1Data.add(
                    new VisionData(
                        new AprilTag(target.getFiducialId(), fiducialPose.get()), 
                        target.getBestCameraToTarget(), 
                        status.getRobotToCamera1(), 
                        status.getCamera1Result().getTimestampSeconds()
                    )
                );
            }
        }
        // Camera 2
        ArrayList<VisionData> camera2Data = new ArrayList<VisionData>();
        if (status.getCamera2Result() != null) {
            for(PhotonTrackedTarget target : status.getCamera2Result().targets) {
                Optional<Pose3d> fiducialPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (fiducialPose.isEmpty()) {continue;}
                
                camera2Data.add(
                    new VisionData(
                        new AprilTag(target.getFiducialId(), fiducialPose.get()), 
                        target.getBestCameraToTarget(), 
                        status.getRobotToCamera2(), 
                        status.getCamera2Result().getTimestampSeconds()
                    )
                );
            }
        }
        ArrayList<VisionData> cameraData = new ArrayList<VisionData>();
        cameraData.addAll(camera1Data);
        cameraData.addAll(camera2Data);

        status.setVisionData(cameraData);
        
        // Limelight
        if(newCommand.getTargetPipeline() != null)
        {
            status.setTargetPipeline(newCommand.getTargetPipeline());
            pipelineStartTimestamp = currentTime;
        }
        else if(status.getTargetPipeline() == LimelightPipeline.Pole)
        {
            status.setTargetPipeline(LimelightPipeline.Cone);
        }

        if(status.getCurrentPipeline() == status.getTargetPipeline())
        {
            switch(status.getCurrentPipeline())
            {
                case Cone:  status.setTargetPipeline(LimelightPipeline.Cube);   break;
                case Cube:  status.setTargetPipeline(LimelightPipeline.Cone);   break;
                default:    break;
            }
            pipelineStartTimestamp = currentTime;
        }

        if(status.getTargetExists())
        {
            switch(status.getCurrentPipeline())
            {
                case Cone:
                    status.setLatestConeXAngle(status.getTargetXAngle());
                    status.setLatestConeYAngle(status.getTargetYAngle());
                    status.setLatestConeArea(status.getCurrentArea());
                    status.setConeExists(status.getTargetExists());
                    break;
                    
                case Cube:
                    status.setLatestCubeXAngle(status.getTargetXAngle());
                    status.setLatestCubeYAngle(status.getTargetYAngle());
                    status.setLatestCubeArea(status.getCurrentArea());
                    status.setCubeExists(status.getTargetExists());
                break;
    
                default: break;
            }
        }
    }

    @Override public void Enabled() {}
    @Override public void Disabled() {}
}
