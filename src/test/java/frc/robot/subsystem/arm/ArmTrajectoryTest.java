package frc.robot.subsystem.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
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
        
        
        double theta0_actual = 1.0;
        double theta1_actual = 2.0;

        ArmTrajectory interpTraj = ArmTrajectory.interpolateStaringPositionError(traj, theta0_actual, theta1_actual);

        // interpTraj should end in the same place
        finalState = interpTraj.getFinalState();
        assertEquals((N-1) * w0, finalState.get(0,0), kEps);
        assertEquals(0.0, finalState.get(0,1), kEps);
        assertEquals(0.0, finalState.get(0,2), kEps);
        assertEquals((N-1) * w1, finalState.get(1,0), kEps);
        assertEquals(0.0, finalState.get(1,1), kEps);
        assertEquals(0.0, finalState.get(1,2), kEps);

        state = interpTraj.sample(0);
        assertEquals(theta0_actual, state.get(0,0), kEps);
        assertEquals(((N-1)*w0-theta0_actual)/totalTime, state.get(0,1), kEps);
        assertEquals(theta1_actual, state.get(1,0), kEps);
        assertEquals(((N-1)*w1-theta1_actual)/totalTime, state.get(1,1), kEps);
        
        state = interpTraj.sample(totalTime/2.0);
        assertEquals((theta0_actual + (N-1) * w0)/2.0, state.get(0,0), kEps);
        assertEquals(((N-1)*w0-theta0_actual)/totalTime, state.get(0,1), kEps);
        assertEquals((theta1_actual + (N-1) * w1)/2.0, state.get(1,0), kEps);
        assertEquals(((N-1)*w1-theta1_actual)/totalTime, state.get(1,1), kEps);
        
        state = traj.sample(totalTime);
        assertEquals(finalState.get(0,0), state.get(0,0), kEps);
        assertEquals(0.0, state.get(0,1), kEps);
        assertEquals(finalState.get(1,0), state.get(1,0), kEps);
        assertEquals(0.0, state.get(1,1), kEps);        

    }        
}
