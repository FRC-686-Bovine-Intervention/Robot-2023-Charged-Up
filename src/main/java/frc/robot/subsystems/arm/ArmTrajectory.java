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

  public static Matrix<N2, N3> getFixedState(double position_0, double position_1) {
    // set posistion_0 and position_1, but zero out the velocity and acceleration terms
    return new MatBuilder<>(Nat.N2(), Nat.N3()).fill(position_0, 0.0, 0.0, position_1, 0.0, 0.0);
  }


  /**
   * Samples the trajectory at a time, returning a matrix with the position, velocities, and
   * accelerations of the joints.
   */
  public Matrix<N2, N3> sample(double time) {
    var dt = getTotalTime() / (points.size() - 1);  // includes grannyFactor

    // Get surrounding points
    int prevIndex = (int) Math.floor(time / dt);
    int nextIndex = (int) Math.ceil(time / dt);
    if (nextIndex == prevIndex) nextIndex++;
    int secondPrevIndex = prevIndex - 1;
    int secondNextIndex = nextIndex + 1;

    // Clamp to allowed indices
    prevIndex = MathUtil.clamp(prevIndex, 0, points.size() - 1);
    nextIndex = MathUtil.clamp(nextIndex, 0, points.size() - 1);
    secondPrevIndex = MathUtil.clamp(secondPrevIndex, 0, points.size() - 1);
    secondNextIndex = MathUtil.clamp(secondNextIndex, 0, points.size() - 1);

    // Calculate positions
    double position_0 =
        MathUtil.interpolate(
          points.get(prevIndex).get(0, 0), points.get(nextIndex).get(0, 0), (time % dt) / dt);
    double position_1 =
        MathUtil.interpolate(
          points.get(prevIndex).get(1, 0), points.get(nextIndex).get(1, 0), (time % dt) / dt);

    // Calculate velocities
    double velocity_0 = (points.get(nextIndex).get(0, 0) - points.get(prevIndex).get(0, 0)) / dt;
    double velocity_1 = (points.get(nextIndex).get(1, 0) - points.get(prevIndex).get(1, 0)) / dt;

    // Calculate accelerations
    double acceleration_0, acceleration_1;
    if ((time % dt) / dt < 0.5) {
      double prevVelocity_0 =
          (points.get(prevIndex).get(0, 0) - points.get(secondPrevIndex).get(0, 0)) / dt;
      double prevVelocity_1 =
          (points.get(prevIndex).get(1, 0) - points.get(secondPrevIndex).get(1, 0)) / dt;
      acceleration_0 = (velocity_0 - prevVelocity_0) / dt;
      acceleration_1 = (velocity_1 - prevVelocity_1) / dt;
    } else {
      double nextVelocity_0 =
          (points.get(secondNextIndex).get(0, 0) - points.get(nextIndex).get(0, 0)) / dt;
      double nextVelocity_1 =
          (points.get(secondNextIndex).get(1, 0) - points.get(nextIndex).get(1, 0)) / dt;
      acceleration_0 = (nextVelocity_0 - velocity_0) / dt;
      acceleration_1 = (nextVelocity_1 - velocity_1) / dt;
    }

    return new MatBuilder<>(Nat.N2(), Nat.N3())
        .fill(position_0, velocity_0, acceleration_0, position_1, velocity_1, acceleration_1);
  }
}
