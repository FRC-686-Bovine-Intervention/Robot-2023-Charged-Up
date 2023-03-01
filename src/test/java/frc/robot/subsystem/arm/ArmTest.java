package frc.robot.subsystem.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmKinematics;
import frc.robot.subsystems.arm.ArmLoop;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmTrajectory;
import frc.robot.subsystems.arm.json.ArmConfigJson;
import frc.robot.subsystems.arm.json.ArmPathsJson;
import frc.robot.subsystems.arm.json.ArmPresetsJson;

public class ArmTest {
    
    private static final double kEps = 1e-6;

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
    public void TestReadArmConfig() {
        // workaround for deploy directory being returned incorrectly
        setDeployDirectoryDuringTest();

        // Get presets from JSON
        File configFile = new File(Filesystem.getDeployDirectory(), ArmConfigJson.jsonFilename);
        ArmConfigJson armConfig = ArmConfigJson.loadJson(configFile);

        assertEquals(24.0, armConfig.frame_width_inches(), kEps);
        assertEquals(30.5, armConfig.bumper_width_inches(), kEps);
        assertEquals(257, armConfig.shoulder().motor().stallCurrentAmps, kEps);
    }


    @Test
    public void TestReadArmPresets() {
        // workaround for deploy directory being returned incorrectly
        setDeployDirectoryDuringTest();

        // Get presets from JSON
        File presetFile = new File(Filesystem.getDeployDirectory(), ArmPresetsJson.jsonFilename);
        ArmPresetsJson presets = ArmPresetsJson.loadJson(presetFile);

        ArmPose.Preset.writePresets(presets);

        assertEquals(0, ArmPose.Preset.DEFENSE.getFileIdx());
        assertEquals(6.0, ArmPose.Preset.DEFENSE.getX(), kEps);
        assertEquals(25.0, ArmPose.Preset.DEFENSE.getZ(), kEps);
        
        assertEquals(0, ArmPose.Preset.DEFENSE.getFileIdx());
        assertEquals(1, ArmPose.Preset.INTAKE.getFileIdx());
        assertEquals(2, ArmPose.Preset.DOUBLE_SUBSTATION.getFileIdx());
        assertEquals(3, ArmPose.Preset.SCORE_HYBRID.getFileIdx());
        assertEquals(4, ArmPose.Preset.SCORE_MID_CUBE.getFileIdx());
        assertEquals(5, ArmPose.Preset.SCORE_HIGH_CUBE.getFileIdx());
        assertEquals(6, ArmPose.Preset.SCORE_MID_CONE.getFileIdx());
        assertEquals(7, ArmPose.Preset.SCORE_HIGH_CONE.getFileIdx());
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

        // Get paths from JSONs
        for (ArmPose.Preset startPos : ArmPose.Preset.values()) {
            for (ArmPose.Preset finalPos : ArmPose.Preset.values()) {
                int startIdx = startPos.getFileIdx();
                int finalIdx = finalPos.getFileIdx();
                String pathFilename = String.format("paths/arm_path_%d_%d.json", startIdx, finalIdx);
                
                File pathFile = new File(Filesystem.getDeployDirectory(), pathFilename);
                paths[startIdx][finalIdx] = ArmPathsJson.loadJson(pathFile);
            }
        }
    
        assertEquals("defense", paths[0][1].startPos());
        assertEquals("intake", paths[0][1].finalPos());
        
        assertEquals("defense", paths[0][0].startPos());
        assertEquals("score_high_cone", paths[0][7].finalPos());

        assertEquals("score_high_cone", paths[7][0].startPos());
        assertEquals("defense", paths[7][0].finalPos());

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
        double minTheta21 = -Math.PI;
        double maxTheta21 = +Math.PI;

        ArmKinematics kinematics = new ArmKinematics(shoulder, l1, l2, minTheta1, maxTheta1, minTheta2, maxTheta2, minTheta21, maxTheta21);

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



    
    Matrix<N2,N3> finalTrajectoryState;

    private final double xMinSetpoint =  0.0; 
    private double xMaxSetpoint;          // calculated
    private final double zMinSetpoint =  0.0; 
    private final double zMaxSetpoint = Units.inchesToMeters(72.0); 
        
    private double xThrottle = 0.0;
    private double zThrottle = 0.0;
    private double xAdjustment = 0.0; // extension from turret center of rotation
    private double zAdjustment = 0.0; // height
    private final double xAdjustmentMaxRangeInches = 24.0;
    private final double zAdjustmentMaxRangeInches = 24.0;
    private final double xAdjustmentMaxRange = Units.inchesToMeters(xAdjustmentMaxRangeInches);
    private final double zAdjustmentMaxRange = Units.inchesToMeters(zAdjustmentMaxRangeInches);

    private final double manualMaxSpeedInchesPerSec = 6.0;    // speed the arm is allowed to extend manually in the turret's XZ plane
    private final double manualMaxSpeedMetersPerSec = Units.inchesToMeters(manualMaxSpeedInchesPerSec);
    private final double manualMaxSpeedDegreesPerSec = 10.0;  // speed the turret is allowed to manually spin

    ArmKinematics kinematics;

    private Matrix<N2,N3> setpointState = null;   

    @Test
    public void TestArmManualAdjustment() {
        // workaround for deploy directory being returned incorrectly
        setDeployDirectoryDuringTest();

        // Get presets from JSON
        File configFile = new File(Filesystem.getDeployDirectory(), ArmConfigJson.jsonFilename);
        ArmConfigJson config = ArmConfigJson.loadJson(configFile);

        kinematics = new ArmKinematics(new Translation2d(config.origin().getX(), config.origin().getY()),
        config.shoulder().length(), config.elbow().length(),
        config.shoulder().minAngle(), config.shoulder().maxAngle(), 
        config.elbow().minAngle(), config.elbow().maxAngle(),
        ArmLoop.kRelativeMinAngleRad, ArmLoop.kRelativeMaxAngleRad);

        xMaxSetpoint = Units.inchesToMeters(config.frame_width_inches() + 48.0);

        // Get presets from JSON
        File presetFile = new File(Filesystem.getDeployDirectory(), ArmPresetsJson.jsonFilename);
        ArmPresetsJson presets = ArmPresetsJson.loadJson(presetFile);
        ArmPose.Preset.writePresets(presets);
        
        double theta1 = ArmPose.Preset.SCORE_HIGH_CONE.getShoulderAngleRad();
        double theta2 = ArmPose.Preset.SCORE_HIGH_CONE.getElbowAngleRad();
        finalTrajectoryState = new MatBuilder<>(Nat.N2(),Nat.N3()).fill(theta1,0,0,theta2,0,0);        


        Vector<N2> startTheta = new Vector<>(finalTrajectoryState.extractColumnVector(0));            
        Translation2d xz = kinematics.forward(startTheta);
        double startX = xz.getX();
        double startZ = xz.getY();            


        double x = 0.0;
        double z = 0.0;

        int numSteps = 10;

        xThrottle = -1.0;
        zThrottle = 0.0;
        for (int k=0; k<numSteps; k++) {
            manualAdjustments();
            
            Vector<N2> setpointTheta = new Vector<>(setpointState.extractColumnVector(0));            
            xz = kinematics.forward(setpointTheta);
            x = xz.getX();
            z = xz.getY();            
        }

        assertEquals(startX + numSteps*xThrottle * manualMaxSpeedMetersPerSec * Constants.loopPeriodSecs, x, kEps);
        assertEquals(startZ + numSteps*zThrottle * manualMaxSpeedMetersPerSec * Constants.loopPeriodSecs, z, kEps);

        xThrottle = 0.0;
        zThrottle = -1.0;
        for (int k=0; k<numSteps; k++) {
            manualAdjustments();
            
            Vector<N2> setpointTheta = new Vector<>(setpointState.extractColumnVector(0));            
            xz = kinematics.forward(setpointTheta);
            x = xz.getX();
            z = xz.getY();            
        }

        assertEquals(startX + numSteps*-1.0 * manualMaxSpeedMetersPerSec * Constants.loopPeriodSecs, x, kEps);
        assertEquals(startZ + numSteps*-1.0 * manualMaxSpeedMetersPerSec * Constants.loopPeriodSecs, z, kEps);

        System.out.println(setpointState);

    }   
    
    
    public void manualAdjustments() {
        // make manual adjustments to final XZ pose
        Vector<N2> thetaFinalTrajectory = new Vector<>(finalTrajectoryState.extractColumnVector(0));
        Translation2d xzFinalTrajectory = kinematics.forward(thetaFinalTrajectory);
        double xFinalTrajectory = xzFinalTrajectory.getX();
        double zfinalTrajectory = xzFinalTrajectory.getY(); // note: Translation2d assumes XY plane, but we are using it in the XZ plane

        // update manual adjustments
        // xThrottle and zThrottle are assumed to be joystick inputs in the range [-1, +1]
        xAdjustment += xThrottle * manualMaxSpeedMetersPerSec * Constants.loopPeriodSecs;
        zAdjustment += zThrottle * manualMaxSpeedMetersPerSec * Constants.loopPeriodSecs;

        // clamp manual adjustments
        xAdjustment = MathUtil.clamp(xAdjustment, -xAdjustmentMaxRange, +xAdjustmentMaxRange);
        zAdjustment = MathUtil.clamp(zAdjustment, -zAdjustmentMaxRange, +zAdjustmentMaxRange);

        // verify frame perimeter
        double xSetpoint = MathUtil.clamp(xFinalTrajectory + xAdjustment, xMinSetpoint, xMaxSetpoint);
        double zSetpoint = MathUtil.clamp(zfinalTrajectory + zAdjustment, zMinSetpoint, zMaxSetpoint);

        // calcualate current manual adjustment after clamping
        xAdjustment = xSetpoint - xFinalTrajectory;
        zAdjustment = zSetpoint - zfinalTrajectory;

        // find new setpoint
        Optional<Vector<N2>> optTheta = kinematics.inverse(xSetpoint, zSetpoint);
        if (optTheta.isPresent()) {
            Vector<N2> setpointTheta = optTheta.get();
            setpointState = ArmTrajectory.getFixedState(setpointTheta);
        } else {
            boolean break_here = true;
        }
    }
}
