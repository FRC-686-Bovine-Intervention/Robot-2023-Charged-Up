package frc.robot.subsystem.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import frc.robot.subsystems.arm.ArmKinematics;

public class ArmKinematicsTest {
    
    private static final double kEps = 1e-9;

    @Test
    public void TestArmKinematics() {
        
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
}
