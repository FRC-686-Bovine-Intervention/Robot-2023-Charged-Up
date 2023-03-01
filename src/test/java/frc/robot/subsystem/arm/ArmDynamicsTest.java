package frc.robot.subsystem.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.File;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.arm.ArmDynamics;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.json.ArmConfigJson;
import frc.robot.subsystems.arm.json.ArmPresetsJson;

public class ArmDynamicsTest {
    
    private static final double kEps = 1e-9;

    @Test
    public void TestDefenseTorque() {
        // workaround for deploy directory being returned incorrectly
        ArmTest.setDeployDirectoryDuringTest();

        // Get presets from JSON
        File configFile = new File(Filesystem.getDeployDirectory(), ArmConfigJson.jsonFilename);
        ArmConfigJson config = ArmConfigJson.loadJson(configFile);
        ArmDynamics dynamics = new ArmDynamics(config.shoulder(), ArmDynamics.rigidlyCombineJoints(config.elbow(), config.wrist()));    
        
        // Get presets from JSON
        File presetFile = new File(Filesystem.getDeployDirectory(), ArmPresetsJson.jsonFilename);
        ArmPresetsJson presets = ArmPresetsJson.loadJson(presetFile);
        ArmPose.Preset.writePresets(presets);

        double theta1 = ArmPose.Preset.DEFENSE.getShoulderAngleRad();
        double theta2 = ArmPose.Preset.DEFENSE.getElbowAngleRad();

        double g = 9.80665;
        double m1 = config.shoulder().mass();
        double l1 = config.shoulder().length();
        double r1 = config.shoulder().cgRadius();
        double m2 = config.elbow().mass();
        double l2 = config.elbow().length();
        double r2 = config.elbow().cgRadius();
        double m3 = config.wrist().mass();
        double r3 = config.wrist().cgRadius();
        
        double expected_torque1 = m1*g*r1*Math.cos(theta1) + 
                                  m2*g*(l1*Math.cos(theta1)+r2*Math.cos(theta2)) + 
                                  m3*g*(l1*Math.cos(theta1)+(l2+r3)*Math.cos(theta2));
        double expected_torque2 = m2*g*(r2*Math.cos(theta2)) + 
                                  m3*g*((l2+r3)*Math.cos(theta2));

        Vector<N2> position =  VecBuilder.fill(theta1, theta2);
        
        var torque = dynamics.Tg(position);

        assertEquals(expected_torque1, torque.get(0,0), kEps);
        assertEquals(expected_torque2, torque.get(1,0), kEps);

        Matrix<N2,N3> state =  new MatBuilder<>(Nat.N2(),Nat.N3()).fill(theta1, 0, 0, theta2, 0, 0);
        var voltages = dynamics.feedforward(state);        
    }
}
