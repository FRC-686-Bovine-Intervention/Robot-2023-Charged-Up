package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.subsystems.framework.LoopBase;

public class VisionLoop extends LoopBase {
    private static VisionLoop instance;
    public static VisionLoop getInstance(){if(instance == null){instance = new VisionLoop();}return instance;}

    private final VisionStatus status = VisionStatus.getInstance();

    private VisionLoop() {Subsystem = Vision.getInstance();}

    @Override
    public void Update() {
        VisionCommand newCommand = status.getCommand();

        // AprilTags
        ArrayList<EstimatedRobotPose> cameraData = new ArrayList<EstimatedRobotPose>();
        for(int i = 0; i < status.getAprilTagCameras().length; i++) {
            PhotonHalHelper cam = status.getAprilTagCamera(i);
            if(cam.getLatestEstimatedPose().isPresent()) {
                cameraData.add(cam.getLatestEstimatedPose().get());
            }
            // PhotonPipelineResult camResult = cam.getLatestCameraResult();
            // Optional<EstimatedRobotPose> estimatedPose = cam.getLatestEstimatedPose();
            // PhotonPipelineResult result = status.getCamResults()[i];
            // if(result != null) {
            //     for(PhotonTrackedTarget target : result.targets) {
            //         Optional<Pose3d> fiducialPose = VisionStatus.aprilTagFieldLayout.getTagPose(target.getFiducialId());
            //         if (fiducialPose.isEmpty()) {continue;}
                    
            //         cameraData.add(
            //             new VisionData(
            //                 new AprilTag(target.getFiducialId(), fiducialPose.get()), 
            //                 target.getBestCameraToTarget(), 
            //                 ArmStatus.getInstance().getRobotToTurret().plus(VisionStatus.turretToCameras[i]), 
            //                 result.getTimestampSeconds()
            //             )
            //         );
            //     }
            // }
        }

        status.setVisionData(cameraData);
        
        // Limelight
        if(newCommand.getTargetPipeline() != null)
        {
            status.setTargetPipeline(newCommand.getTargetPipeline());
        }
        // else if(status.getTargetPipeline() == LimelightPipeline.Pole)
        // {
        //     status.setTargetPipeline(LimelightPipeline.Cone);
        // }

        // if(status.getCurrentPipeline() == status.getTargetPipeline())
        // {
        //     switch(status.getCurrentPipeline())
        //     {
        //         case Cone:  status.setTargetPipeline(LimelightPipeline.Cube);   break;
        //         case Cube:  status.setTargetPipeline(LimelightPipeline.Cone);   break;
        //         default:    break;
        //     }
        // }

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
