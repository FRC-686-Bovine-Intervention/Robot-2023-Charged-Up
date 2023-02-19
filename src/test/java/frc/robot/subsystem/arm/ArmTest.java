package frc.robot.subsystem.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.arm.ArmKinematics;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.json.ArmPathsJson;
import frc.robot.subsystems.arm.json.ArmPresetsJson;

public class ArmTest {
    
    private static final double kEps = 1e-9;

    /** Workaround for deploy directory being returned incorrectly during JUnit test */
    public void setDeployDirectoryDuringTest() {
        File deployDir = Filesystem.getDeployDirectory();

        // remove everything starting at /build
        String userDir = deployDir.getPath();
        int idx = userDir.indexOf(File.separator + "build");
        if (idx > 0) {
            userDir = userDir.substring(0, idx);
            System.setProperty("user.dir", userDir);
        }
    }


    @Test
    public void TestReadArmPresets() {
        // workaround for deploy directory being returned incorrectly
        setDeployDirectoryDuringTest();

        // Get presets from JSON
        File presetFile = new File(Filesystem.getDeployDirectory(), ArmPresetsJson.jsonFilename);
        ArmPresetsJson presets = ArmPresetsJson.loadJson(presetFile);

        ArmPose.Preset.writePresets(presets);

        assertEquals(1, ArmPose.Preset.DEFENSE.getFileIdx());
        assertEquals(0.0, ArmPose.Preset.DEFENSE.getX(), kEps);
        assertEquals(25.0, ArmPose.Preset.DEFENSE.getY(), kEps);
        
        assertEquals(1, ArmPose.Preset.DEFENSE.getFileIdx());
        assertEquals(2, ArmPose.Preset.INTAKE.getFileIdx());
        assertEquals(3, ArmPose.Preset.DOUBLE_SUBSTATION.getFileIdx());
        assertEquals(4, ArmPose.Preset.SCORE_HYBRID.getFileIdx());
        assertEquals(5, ArmPose.Preset.SCORE_MID_CUBE.getFileIdx());
        assertEquals(6, ArmPose.Preset.SCORE_HIGH_CUBE.getFileIdx());
        assertEquals(7, ArmPose.Preset.SCORE_MID_CONE.getFileIdx());
        assertEquals(8, ArmPose.Preset.SCORE_HIGH_CONE.getFileIdx());
    }

    @Test
    public void TestReadArmPaths() {
        // workaround for deploy directory being returned incorrectly
        setDeployDirectoryDuringTest();        

        // Get presets from JSON
        File presetFile = new File(Filesystem.getDeployDirectory(), ArmPresetsJson.jsonFilename);
        ArmPresetsJson presets = ArmPresetsJson.loadJson(presetFile);   
        ArmPose.Preset.writePresets(presets);

        final ArmPathsJson paths[][] = new ArmPathsJson[ArmPose.Preset.values().length+1][ArmPose.Preset.values().length+1];

        // Get paths from JSON
        for (ArmPose.Preset startPos : ArmPose.Preset.values()) {
            for (ArmPose.Preset finalPos : ArmPose.Preset.values()) {
                int startIdx = startPos.getFileIdx();
                int finalIdx = finalPos.getFileIdx();
                String pathFilename = String.format("paths/arm_path_%d_%d.json", startIdx, finalIdx);
                
                File pathFile = new File(Filesystem.getDeployDirectory(), pathFilename);
                paths[startIdx][finalIdx] = ArmPathsJson.loadJson(pathFile);
            }
        }
    
        assertEquals("defense", paths[1][1].startPos());
        assertEquals("score_high_cone", paths[1][8].finalPos());
        
        assertEquals("score_high_cone", paths[8][1].startPos());
        assertEquals("defense", paths[8][1].finalPos());

    }

    @Test
    public void TestArmKinematics() {
        // workaround for deploy directory being returned incorrectly
        setDeployDirectoryDuringTest();
        
        Translation2d shoulder = new Translation2d(1.0, 1.0);
        double l1 = 1.2;
        double l2 = 1.4;
        double minTheta1 = -Math.PI;
        double maxTheta1 = +Math.PI;
        double minTheta2 = -Math.PI;
        double maxTheta2 = +Math.PI;

        ArmKinematics kinematics = new ArmKinematics(shoulder, l1, l2, minTheta1, maxTheta1, minTheta2, maxTheta2);

        double theta1 = -0.3;
        double theta2 = +1.0;
        Vector<N2> theta = VecBuilder.fill(theta1, theta2);

        Translation2d endEffector = kinematics.forward(theta);

        Optional<Vector<N2>> optTheta = kinematics.inverse(endEffector);

        // check that we come back to the same spot
        assertTrue(optTheta.isPresent());
        Vector<N2> newTheta = optTheta.get();
        assertEquals(theta1, newTheta.get(0,0), kEps);
        assertEquals(theta2, newTheta.get(1,0), kEps);


        // now check that inverse kinematics doesn't work if the target is too far away
        endEffector = new Translation2d(10.0, 0.0);

        optTheta = kinematics.inverse(endEffector);

        // check that we come back to the same spot
        assertFalse(optTheta.isPresent());

    }
}
