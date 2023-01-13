package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.framework.StatusBase;
import io.github.oblarg.oblog.annotations.Log;

public class VisionStatus extends StatusBase {
    public static VisionStatus instance;
    public static VisionStatus getInstance(){
        if(instance == null){
            instance = new VisionStatus();
        }
        return instance;
    }

    private VisionStatus()
    {
        Subsystem = Vision.getInstance();
    }

    private final VisionHAL HAL = VisionHAL.getInstance();

    //@Log(name = "Vision Pose")
    private Pose3d visionPose = new Pose3d();
    public Pose3d getVisionPose() {return visionPose;}
    public VisionStatus setVisionPose(Pose3d visionPose) {
        this.visionPose = visionPose;
        return this;
    }

    @Override
    public void exportToTable(LogTable table, String prefix) {
        double[] deconstructedPose = new double[]{
            visionPose.getX(),
            visionPose.getY(),
            visionPose.getZ(),
            visionPose.getRotation().getQuaternion().getW(),
            visionPose.getRotation().getQuaternion().getX(),
            visionPose.getRotation().getQuaternion().getY(),
            visionPose.getRotation().getQuaternion().getZ()
        };
        table.put(prefix + "/Vision Pose", deconstructedPose);
    }

    @Override
    public void importFromTable(LogTable table, String prefix) {
        double[] deconstructedPose = table.getDoubleArray(prefix + "/Vision Pose", null);
        setVisionPose(new Pose3d(new Translation3d(deconstructedPose[0],deconstructedPose[1],deconstructedPose[2]), new Rotation3d(new Quaternion(deconstructedPose[3],deconstructedPose[4],deconstructedPose[5],deconstructedPose[6]))));
    }

    @Override
    public void updateInputs() {
        Pose3d estimatedPose = HAL.getEstimatedGlobalPose(visionPose);
        if(estimatedPose != null)
            setVisionPose(estimatedPose);
    }

    @Override
    public void recordOutputs(String prefix) {
        // TODO Auto-generated method stub
        
    }
    
}
