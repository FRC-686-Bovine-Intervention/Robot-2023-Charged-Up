package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonHalHelper {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonHalHelper(PhotonCamera camera, AprilTagFieldLayout aprilTagLayout) {this(camera, new Transform3d(), PoseStrategy.MULTI_TAG_PNP, aprilTagLayout);}
    public PhotonHalHelper(PhotonCamera camera, Transform3d robotToCamera, PoseStrategy poseStrategy, AprilTagFieldLayout aprilTagLayout) {
        this.camera = camera;
        this.poseEstimator = new PhotonPoseEstimator(aprilTagLayout, poseStrategy, camera, robotToCamera);
    }

    private PhotonPipelineResult    latestCameraResult = new PhotonPipelineResult();
    public PhotonPipelineResult     getLatestCameraResult() {return latestCameraResult;}
    
    private Optional<EstimatedRobotPose>    latestEstimatedPose = Optional.empty();
    public Optional<EstimatedRobotPose>     getLatestEstimatedPose() {return latestEstimatedPose;}

    public PhotonHalHelper updateInputs() {
        if(camera != null) {
            latestCameraResult = camera.getLatestResult();
        }
        return this;
    }

    public PhotonHalHelper exportToTable(LogTable table) {
        if(latestCameraResult != null) {
            table.put(camera.getName(), latestCameraResult.populatePacket(new Packet(latestCameraResult.getPacketSize())).getData());
        }
        return this;
    }

    public PhotonHalHelper importFromTable(LogTable table) {
        Packet camPacket = new Packet(table.getRaw(camera.getName(), new byte[0]));
        if(camPacket.getSize() <= 0) {
            latestCameraResult = new PhotonPipelineResult();
            latestCameraResult.createFromPacket(camPacket);
        }
        return this;
    }

    public PhotonHalHelper processTable() {
        if(latestCameraResult != null) {
            latestEstimatedPose = poseEstimator.update(latestCameraResult);
        }
        return this;
    }

    public PhotonHalHelper processOutputs(Logger logger, String prefix) {
        

        return this;
    }

    public AprilTagFieldLayout getAprilTagFieldLayout() {return poseEstimator.getFieldTags();}

    public PhotonHalHelper setCameraTransform(Transform3d robotToCamera) {
        poseEstimator.setRobotToCameraTransform(robotToCamera);
        return this;
    }
}
