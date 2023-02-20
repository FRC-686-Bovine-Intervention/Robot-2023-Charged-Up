// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

/** Represents a trajectory of arm states that can be generated asynchronously. */
public class ArmTrajectory {
  private final String startPos;   // starting position in XZ coordinates
  private final String finalPos;   // final position in XZ coordinates
  private double totalTime = 0.0;         // total trajectory time
  private List<Vector<N2>> points = new ArrayList<>();  // rough trajectory of theta1, theta2 in equally spaced times across totalTime
  Matrix<N2, N3> finalState;

  /** Creates an arm trajectory with the given parameters. */
  public ArmTrajectory(String startPos, String finalPos, double totalTime, List<Vector<N2>> points) {
    this.startPos = startPos;
    this.finalPos = finalPos;
    this.totalTime = totalTime;
    this.points = points;

    // precalculate the final state, as we will use this to hold position after completing the trajectory
    this.finalState = getFixedState(points.get(points.size()-1));
  }

  /** slow down factor for arm movements */
  private double grannyFactor = 1.0;  // default to full speed motions

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
  
  /** Returns the total time for the trajectory, possibly lengthened by GrannyFactor. */
  public double getTotalTime() {
    return this.totalTime * this.grannyFactor;
  }

  /** Returns the generated interior points. */
  public List<Vector<N2>> getPoints() {
    return this.points;
  }

  /** Returns the final state */
  public Matrix<N2, N3> getFinalState() {
    return this.finalState;
  }

  public static Matrix<N2, N3> getFixedState(Vector<N2> position) {
    // set posistion_0 and position_1, but zero out the velocity and acceleration terms
    return getFixedState(position.get(0,0), position.get(1,0));
  }

  public static Matrix<N2, N3> getFixedState(double position0, double position1) {
    // set posistion_0 and position_1, but zero out the velocity and acceleration terms
    return new MatBuilder<>(Nat.N2(), Nat.N3()).fill(position0, 0.0, 0.0, position1, 0.0, 0.0);
  }


  /**
   * Samples the trajectory at a time, returning a matrix with the position, velocities, and
   * accelerations of the joints.
   */
  public Matrix<N2, N3> sample(double time) {
    int N = points.size();
    var dt = getTotalTime() / (N-1);  // includes grannyFactor

    // Get surrounding points
    int iPrev1 = (int) Math.floor(time / dt);
    int iNext1 = (int) Math.ceil(time / dt);
    if (iNext1 == iPrev1) iNext1++;
    int iPrev2 = iPrev1 - 1;
    int iNext2 = iNext1 + 1;

    // Clamp to allowed indices
    iPrev1 = MathUtil.clamp(iPrev1, 0, N-1);
    iNext1 = MathUtil.clamp(iNext1, 0, N-1);
    iPrev2 = MathUtil.clamp(iPrev2, 0, N-1);
    iNext2 = MathUtil.clamp(iNext2, 0, N-1);

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
}
