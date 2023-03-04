package frc.robot.subsystem.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.arm.ArmTrajectory;

public class ArmTrajectoryTest {
    
    private static final double kEps = 1e-9;

    @Test
    public void TestArmTrajectory() {
        
        double w0 = 0.1;
        double w1 = 0.2;

        List<Vector<N2>> points = new ArrayList<>();
        int N = 50; // number of points
        for (int k=0; k<N; k++) {
            double theta0 = w0 * k;
            double theta1 = w1 * k;
            points.add(VecBuilder.fill(theta0, theta1));
        }

        double totalTime = 1.0;
        ArmTrajectory traj = new ArmTrajectory("begin", "end", totalTime, points);

        assertEquals(totalTime, traj.getTotalTime(), kEps);

        double grannyFactor = 5.0;
        traj.setGrannyFactor(grannyFactor);
        assertEquals(totalTime * grannyFactor, traj.getTotalTime(), kEps);

        grannyFactor = 1.0;
        traj.setGrannyFactor(grannyFactor);
        assertEquals(totalTime * grannyFactor, traj.getTotalTime(), kEps);
        
        var finalState = traj.getFinalState();
        assertEquals((N-1) * w0, finalState.get(0,0), kEps);
        assertEquals(0.0, finalState.get(0,1), kEps);
        assertEquals(0.0, finalState.get(0,2), kEps);
        assertEquals((N-1) * w1, finalState.get(1,0), kEps);
        assertEquals(0.0, finalState.get(1,1), kEps);
        assertEquals(0.0, finalState.get(1,2), kEps);

        var state = traj.sample(totalTime/2.0);
        assertEquals((N-1)/2.0 * w0, state.get(0,0), kEps);
        assertEquals((N-1)*w0/totalTime, state.get(0,1), kEps);
        assertEquals(0.0, state.get(0,2), kEps);
        assertEquals((N-1)/2.0 * w1, state.get(1,0), kEps);
        assertEquals((N-1)*w1/totalTime, state.get(1,1), kEps);
        assertEquals(0.0, state.get(1,2), kEps);
     
        state = traj.sample(totalTime);
        assertEquals(finalState.get(0,0), state.get(0,0), kEps);
        assertEquals(0.0, state.get(0,1), kEps);
        assertEquals(finalState.get(1,0), state.get(1,0), kEps);
        assertEquals(0.0, state.get(1,1), kEps);
        
        
        double start_theta0_actual =  0.01;
        double start_theta1_actual = -0.02;

        assertTrue(traj.startIsNear(start_theta0_actual, start_theta1_actual, Units.degreesToRadians(10.0)));

        start_theta0_actual = 1.0;
        start_theta1_actual = 2.0;

        assertFalse(traj.startIsNear(start_theta0_actual, start_theta1_actual, Units.degreesToRadians(10.0)));


        ArmTrajectory interpTraj = traj.interpolateEndPoints(start_theta0_actual, start_theta1_actual, null, null);

        // interpTraj should end in the same place
        finalState = interpTraj.getFinalState();
        assertEquals((N-1) * w0, finalState.get(0,0), kEps);
        assertEquals(0.0, finalState.get(0,1), kEps);
        assertEquals(0.0, finalState.get(0,2), kEps);
        assertEquals((N-1) * w1, finalState.get(1,0), kEps);
        assertEquals(0.0, finalState.get(1,1), kEps);
        assertEquals(0.0, finalState.get(1,2), kEps);

        state = interpTraj.sample(0);
        assertEquals(start_theta0_actual, state.get(0,0), kEps);
        assertEquals(((N-1)*w0-start_theta0_actual)/totalTime, state.get(0,1), kEps);
        assertEquals(start_theta1_actual, state.get(1,0), kEps);
        assertEquals(((N-1)*w1-start_theta1_actual)/totalTime, state.get(1,1), kEps);
        
        state = interpTraj.sample(totalTime/2.0);
        assertEquals((start_theta0_actual + (N-1) * w0)/2.0, state.get(0,0), kEps);
        assertEquals(((N-1)*w0-start_theta0_actual)/totalTime, state.get(0,1), kEps);
        assertEquals((start_theta1_actual + (N-1) * w1)/2.0, state.get(1,0), kEps);
        assertEquals(((N-1)*w1-start_theta1_actual)/totalTime, state.get(1,1), kEps);
        
        state = interpTraj.sample(totalTime);
        assertEquals(finalState.get(0,0), state.get(0,0), kEps);
        assertEquals(0.0, state.get(0,1), kEps);
        assertEquals(finalState.get(1,0), state.get(1,0), kEps);
        assertEquals(0.0, state.get(1,1), kEps);        

        double final_theta0_actual = 2.0;
        double final_theta1_actual = 1.0;

        interpTraj = traj.interpolateEndPoints(null, null, final_theta0_actual, final_theta1_actual);

        // interpTraj should finish at the actual
        finalState = interpTraj.getFinalState();
        assertEquals(final_theta0_actual, finalState.get(0,0), kEps);
        assertEquals(0.0, finalState.get(0,1), kEps);
        assertEquals(0.0, finalState.get(0,2), kEps);
        assertEquals(final_theta1_actual, finalState.get(1,0), kEps);
        assertEquals(0.0, finalState.get(1,1), kEps);
        assertEquals(0.0, finalState.get(1,2), kEps);

        state = interpTraj.sample(0);
        assertEquals(0.0, state.get(0,0), kEps);
        assertEquals(final_theta0_actual/totalTime, state.get(0,1), kEps);
        assertEquals(0.0, state.get(1,0), kEps);
        assertEquals(final_theta1_actual/totalTime, state.get(1,1), kEps);
        
        state = interpTraj.sample(totalTime/2.0);
        assertEquals(final_theta0_actual/2.0, state.get(0,0), kEps);
        assertEquals(final_theta0_actual/totalTime, state.get(0,1), kEps);
        assertEquals(final_theta1_actual/2.0, state.get(1,0), kEps);
        assertEquals(final_theta1_actual/totalTime, state.get(1,1), kEps);
        
        state = interpTraj.sample(totalTime);
        assertEquals(final_theta0_actual, state.get(0,0), kEps);
        assertEquals(0.0, state.get(0,1), kEps);
        assertEquals(final_theta1_actual, state.get(1,0), kEps);
        assertEquals(0.0, state.get(1,1), kEps);        

    } 
    
    
    @Test
    void startTrajectoryTest() {

        ArmPose.Preset startPos = ArmPose.Preset.DEFENSE;
        ArmPose.Preset finalPos = ArmPose.Preset.SCORE_HIGH_CONE;
        ArmStatus.NodeEnum targetNode = ArmStatus.NodeEnum.TopLeft;
        Pose3d turretPose = new Pose3d();


        public void startTrajectory(ArmPose.Preset startPos, ArmPose.Preset finalPos, ArmStatus.NodeEnum targetNode, Pose3d turretPose) {


    }

}
