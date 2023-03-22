package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonHalHelper {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonHalHelper(PhotonCamera camera, AprilTagFieldLayout aprilTagLayout) {this(camera, new Transform3d(), PoseStrategy.MULTI_TAG_PNP, aprilTagLayout);}
    public PhotonHalHelper(PhotonCamera camera, Transform3d robotToCamera, PoseStrategy poseStrategy, AprilTagFieldLayout aprilTagLayout) {
        this.camera = camera;
        this.poseEstimator = new PhotonPoseEstimator(aprilTagLayout, poseStrategy, camera, robotToCamera);
    }

    public PhotonPipelineResult latestCameraResult;

    public PhotonHalHelper updateInputs() {
        latestCameraResult = camera.getLatestResult();
        return this;
    }

    public PhotonHalHelper exportToTable(LogTable table, String prefix) {
        if(latestCameraResult != null) {
            table.put(prefix, latestCameraResult.populatePacket(new Packet(latestCameraResult.getPacketSize())).getData());
        }
        return this;
    }

    public PhotonHalHelper setCameraTransform(Transform3d robotToCamera) {
        poseEstimator.setRobotToCameraTransform(robotToCamera);
        return this;
    }
}
