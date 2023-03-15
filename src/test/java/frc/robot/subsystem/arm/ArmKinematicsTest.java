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


    @Test
    public void TestArmKinematics2() {

        // debug discrepancy in measured kinematics

        Translation2d shoulder = new Translation2d(13.5 / 39.3701, 50.0 / 39.3701);
        double l1 = 27.0 / 39.3701;
        double l2 = 20.5 / 39.3701;
        double minTheta1 = -Math.PI;
        double maxTheta1 = +Math.PI;
        double minTheta2 = -Math.PI;
        double maxTheta2 = +Math.PI;
        double minTheta21 = -Math.PI;
        double maxTheta21 = +Math.PI;

        ArmKinematics kinematics = new ArmKinematics(shoulder, l1, l2, minTheta1, maxTheta1, minTheta2, maxTheta2, minTheta21, maxTheta21);

        double x = 52.0 / 39.3701;
        double y = 48.5 / 39.3701;

        Translation2d endEffector = new Translation2d(x, y);
        Optional<Vector<N2>> optTheta = kinematics.inverse(endEffector);

        assertTrue(optTheta.isPresent());
        
        Vector<N2> theta = optTheta.get();

        // TROUBLE!  Measured angles do not match measured X,Z
        // assertEquals(-0.667, theta.get(0,0), 0.1);
        // assertEquals(+0.838, theta.get(1,0), 0.1);
        

        // check that we come back to the same spot
        endEffector = kinematics.forward(theta);

        assertEquals(x, endEffector.getX(), kEps);
        assertEquals(y, endEffector.getY(), kEps);
    }
}
