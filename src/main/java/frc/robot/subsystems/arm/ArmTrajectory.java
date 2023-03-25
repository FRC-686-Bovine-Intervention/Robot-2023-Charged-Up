// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

/**
 * Represents a trajectory of arm states that can be generated asynchronously.
 */
public class ArmTrajectory {
  private final String startPos;    // starting position
  private final String finalPos;    // final position
  private double totalTime = 0.0;   // total trajectory time
  private double grannyFactor = 1.0;   // totalTime multiplier in JSON
  private List<Vector<N2>> points = new ArrayList<>(); // rough trajectory of theta1, theta2 at equally spaced times across totalTime
  Matrix<N2, N3> finalState;

  /** Creates an arm trajectory with the given parameters. */
  public ArmTrajectory(String startPos, String finalPos, double totalTime, double grannyFactor, List<Vector<N2>> points) {
    this.startPos = startPos;
    this.finalPos = finalPos;
    this.totalTime = totalTime;
    this.grannyFactor = grannyFactor;
    this.points = points;

    // precalculate the final state, as we will use this to hold position after
    // completing the trajectory
    this.finalState = getFixedState(points.get(points.size() - 1));
  }

  /** slow down factor for arm movements 
   * CHANGE IN AUTOMANAGERLOOP OOOOOOOOOOOOONNNNNNNNNNNNNNLLLLLLLLLLLLLYYYYYYYYYYYYY
  */
  private static double globalGrannyFactor = 1.0;   // change in AutoManagerLoop to change speed in auto/teleop 

  public double getGlobalGrannyFactor() {
    return globalGrannyFactor;
  }

  public static void setGlobalGrannyFactor(double grannyFactor) {
    ArmTrajectory.globalGrannyFactor = MathUtil.clamp(grannyFactor, 1.0, 10.0);
  }

  public double getGrannyFactor() {
    return grannyFactor;
  }

  public void setGrannyFactor(double grannyFactor) {
    this.grannyFactor = MathUtil.clamp(grannyFactor, 1.0, 10.0);
  }

  /** get start position string */
  public String getStartString() {
    return startPos;
  }

  /** set final position string */
  public String getFinalString() {
    return finalPos;
  }

  /**Returns the total time for the trajectory, possibly lengthened by GrannyFactor. */
  public double getTotalTime() {
    return totalTime * grannyFactor * globalGrannyFactor;
  }

  /** Returns the generated interior points. */
  public List<Vector<N2>> getPoints() {
    return points;
  }

  /** Returns the final state */
  public Matrix<N2, N3> getFinalState() {
    return finalState;
  }

  public static Matrix<N2, N3> getFixedState(Vector<N2> position) {
    // set posistion_0 and position_1, but zero out the velocity and acceleration terms
    return getFixedState(position.get(0, 0), position.get(1, 0));
  }

  public static Matrix<N2, N3> getFixedState(double position0, double position1) {
    // set posistion_0 and position_1, but zero out the velocity and acceleration terms
    return new MatBuilder<>(Nat.N2(), Nat.N3()).fill(position0, 0.0, 0.0, position1, 0.0, 0.0);
  }

  /**
   * Samples the trajectory at a time, returning a matrix with the 
   * position, velocities, and accelerations of the joints.
   */
  public Matrix<N2, N3> sample(double time) {
    int N = points.size();
    var dt = getTotalTime() / (N - 1); // includes grannyFactor

    // Get surrounding points
    int iPrev1 = (int) Math.floor(time / dt);
    int iNext1 = (int) Math.ceil(time / dt);
    if (iNext1 == iPrev1)
      iNext1++;
    int iPrev2 = iPrev1 - 1;
    int iNext2 = iNext1 + 1;

    // Clamp to allowed indices
    iPrev1 = MathUtil.clamp(iPrev1, 0, N - 1);
    iNext1 = MathUtil.clamp(iNext1, 0, N - 1);
    iPrev2 = MathUtil.clamp(iPrev2, 0, N - 1);
    iNext2 = MathUtil.clamp(iNext2, 0, N - 1);

    // get neighboring trajectory points
    double theta0_prev2 = points.get(iPrev2).get(0, 0);
    double theta1_prev2 = points.get(iPrev2).get(1, 0);
    double theta0_prev1 = points.get(iPrev1).get(0, 0);
    double theta1_prev1 = points.get(iPrev1).get(1, 0);
    double theta0_next1 = points.get(iNext1).get(0, 0);
    double theta1_next1 = points.get(iNext1).get(1, 0);
    double theta0_next2 = points.get(iNext2).get(0, 0);
    double theta1_next2 = points.get(iNext2).get(1, 0);

    // Calculate positions
    double position0 = MathUtil.interpolate(theta0_prev1, theta0_next1, (time % dt) / dt);
    double position1 = MathUtil.interpolate(theta1_prev1, theta1_next1, (time % dt) / dt);

    // Calculate velocities
    double velocity0 = (theta0_next1 - theta0_prev1) / dt;
    double velocity1 = (theta1_next1 - theta1_prev1) / dt;

    // Calculate accelerations
    double acceleration0, acceleration1;
    if ((time % dt) / dt < 0.5) {
      double velocity0_prev = (theta0_prev1 - theta0_prev2) / dt;
      double velocity1_prev = (theta1_prev1 - theta1_prev2) / dt;
      acceleration0 = (velocity0 - velocity0_prev) / dt;
      acceleration1 = (velocity1 - velocity1_prev) / dt;
    } else {
      double velocity0_next = (theta0_next2 - theta0_next1) / dt;
      double velocity1_next = (theta1_next2 - theta1_next1) / dt;
      acceleration0 = (velocity0_next - velocity0) / dt;
      acceleration1 = (velocity1_next - velocity1) / dt;
    }

    return new MatBuilder<>(Nat.N2(), Nat.N3())
        .fill(position0, velocity0, acceleration0, position1, velocity1, acceleration1);
  }

  public ArmTrajectory interpolateEndPoints(double start_theta0, double start_theta1, double final_theta0,
      double final_theta1) {

      boolean reversePath = false;
      double score_theta0, score_theta1;
      if ((startPos.equals("defense")) && (finalPos.contains("cube") || finalPos.contains("cone"))) {
        reversePath = false;
        score_theta0 = final_theta0;
        score_theta1 = final_theta1;
      } else if ((finalPos.equals("defense")) && (startPos.contains("cube") || startPos.contains("cone"))) {
        reversePath = true;
        score_theta0 = start_theta0;
        score_theta1 = start_theta1;
      } else {
        return this;
      }

      List<Vector<N2>> newPoints = new ArrayList<Vector<N2>>(points);
      double newTotalTime = totalTime;

      if (reversePath) {
        // reverse path so scoring position is always at the back end
        Collections.reverse(newPoints);
      }

      // we will split the path at the maximum shoulder angle.  find it
      double max_theta0 = -Double.MAX_VALUE;
      int idx = 0;
      for (int k=0; k<newPoints.size(); k++) {
        double theta0 = newPoints.get(k).get(0,0);
        if (theta0 > max_theta0) {
          max_theta0 = theta0;
          idx = k;
        }
      }

      // calculate total distance to move final trajectory points
      double delta_theta0 = score_theta0 - newPoints.get(newPoints.size()-1).get(0,0);
      double delta_theta1 = score_theta1 - newPoints.get(newPoints.size()-1).get(1,0);
      // Vector<N2> delta_theta = VecBuilder.fill(delta_theta0, delta_theta1);
      Vector<N2> delta_theta = VecBuilder.fill(delta_theta0, delta_theta1);
      double delta_dist = delta_theta.norm();

      // calculate speed of arm at interpolation point
      double v_theta0 = newPoints.get(idx).get(0,0) - newPoints.get(idx-1).get(0,0);
      double v_theta1 = newPoints.get(idx).get(1,0) - newPoints.get(idx-1).get(1,0);
      double v = Math.sqrt(v_theta0*v_theta0 + v_theta1*v_theta1);

      // calculate number interpolation points to add to list
      int npoints = (int)Math.ceil(delta_dist / v);

      // new path from 0 to idx-1 will be the same

      // new path from idx to end will be moved by delta_theta
      for (int k=idx; k<newPoints.size(); k++) {
        newPoints.set(k, VecBuilder.fill(newPoints.get(k).get(0,0) + delta_theta0,
                                         newPoints.get(k).get(1,0) + delta_theta1));
      }

      // now linearly interpolate between these two segments
      for (int k=1; k<=npoints; k++) {
        double alpha = (double)k/(npoints+1);
        double interp_theta0 = newPoints.get(idx-1).get(0,0) + alpha*delta_theta0;
        double interp_theta1 = newPoints.get(idx-1).get(1,0) + alpha*delta_theta1;
        newPoints.add(idx-1+k, VecBuilder.fill(interp_theta0, interp_theta1));
      }

      newTotalTime += npoints * Constants.loopPeriodSecs;

    if (reversePath) {
        // undo the earlier reversal
        Collections.reverse(newPoints);
    }

    return new ArmTrajectory(startPos, finalPos, newTotalTime, grannyFactor, newPoints);
  }


  
  public boolean startIsNear(double theta0_actual, double theta1_actual, double threshold) {
    Matrix<N2, N1> theta_actual = VecBuilder.fill(theta0_actual, theta1_actual);
    Matrix<N2, N1> theta_error = theta_actual.minus(points.get(0));

    return (theta_error.maxAbs() < threshold);
  }

}
