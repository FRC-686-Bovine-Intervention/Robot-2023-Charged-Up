// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Converts between the system state and motor voltages for a double jointed arm.
 *
 * <p>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 *
 * <p>https://www.chiefdelphi.com/t/double-jointed-arm-physics-control-simulator/424307
 */
public class ArmDynamics {
  private final JointConfig proximal;
  private final JointConfig distal;
  private static final double g = 9.80665;

  /** Physics constants for a single joint. */
  public static class JointConfig {
      double mass;
      double length;
      double moi;
      double cgRadius;
      double minAngle;
      double maxAngle;
      double reduction;
      DCMotor motor;

    public JointConfig(double mass, double length, double moi, double cgRadius, double minAngle, double maxAngle,
    double reduction, DCMotor motor) {
      this.mass = mass;
      this.length = length;
      this.moi = moi;
      this.cgRadius = cgRadius;
      this.minAngle = minAngle;
      this.maxAngle = maxAngle;
      this.reduction = reduction;
      this.motor = motor;
    }

    public double mass() {
      return mass;
    }

    public double length() {
      return length;
    }

    public double moi() {
      return moi;
    }

    public double cgRadius() {
      return cgRadius;
    }

    public double minAngle() {
      return minAngle;
    }

    public double maxAngle() {
      return maxAngle;
    }

    public double reduction() {
      return reduction;
    }

    public DCMotor motor() {
      return motor;
    }

    
  }

  public ArmDynamics(JointConfig proximal, JointConfig distal) {
    this.proximal = proximal;
    this.distal = distal;
  }

  public static JointConfig rigidlyCombineJoints(JointConfig proximal, JointConfig distal) {
    // Rigidly combine a distal joint to a proximal joint
    var combinedCgRadius =
        (proximal.cgRadius() * proximal.mass()
                + (proximal.length() + distal.cgRadius()) * distal.mass())
            / (proximal.mass() + distal.mass());
    var combinedMoi =
        proximal.mass() * Math.pow(proximal.cgRadius() - combinedCgRadius, 2.0)
            + distal.mass()
                * Math.pow(
                    proximal.length() + distal.cgRadius() - combinedCgRadius, 2.0);
    JointConfig combined =
        new JointConfig(
            proximal.mass + distal.mass,
            proximal.length + distal.length,
            combinedMoi,
            combinedCgRadius,
            proximal.minAngle(),
            proximal.maxAngle(),
            proximal.reduction(),
            proximal.motor());

    return combined;
  }

  public boolean isGoodShoulderAngle(double angle, double extraThreshold) {
    return (angle >= proximal.minAngle) && (angle <= proximal.maxAngle + extraThreshold);
  }

  public boolean isGoodElbowAngle(double angle, double extraThreshold) {
    return (angle >= distal.minAngle - extraThreshold) && (angle <= distal.maxAngle + extraThreshold);
  }

  /** Calculates the joint voltages based on the joint positions (feedforward). */
  public Vector<N2> feedforward(Vector<N2> position) {
    return feedforward(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
  }

  /**
   * Calculates the joint voltages based on the full joint states as a matrix (feedforward). The
   * rows represent each joint and the columns represent position, velocity, and acceleration.
   */
  public Vector<N2> feedforward(Matrix<N2, N3> state) {
    return feedforward(
        new Vector<>(state.extractColumnVector(0)),
        new Vector<>(state.extractColumnVector(1)),
        new Vector<>(state.extractColumnVector(2)));
  }

  /** Calculates the joint voltages based on the full joint states as vectors (feedforward). */
  public Vector<N2> feedforward(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
    var torque =
        M(position)
            .times(acceleration)
            .plus(C(position, velocity).times(velocity))
            .plus(Tg(position));
    return VecBuilder.fill(
        proximal.motor().getVoltage(torque.get(0, 0), velocity.get(0, 0)),
        distal.motor().getVoltage(torque.get(1, 0), velocity.get(1, 0)));
  }

  /**
   * Adjusts the simulated state of the arm based on applied voltages.
   *
   * @param state The current state of the arm as (position_0, position_1, velocity_0, velocity_1)
   * @param voltage The applied voltage of each joint.
   * @param dt The step length in seconds.
   * @return The new state of the arm as (position_0, position_1, velocity_0, velocity_1)
   */
  public Vector<N4> simulate(Vector<N4> state, Vector<N2> voltage, double dt) {
    return new Vector<>(
        NumericalIntegration.rkdp(
            (Matrix<N4, N1> x, Matrix<N2, N1> u) -> {
              // x = current state, u = voltages, return = state derivatives

              // Get vectors from state
              var position = VecBuilder.fill(x.get(0, 0), x.get(1, 0));
              var velocity = VecBuilder.fill(x.get(2, 0), x.get(3, 0));

              // Calculate torque
              var shoulderTorque =
                  proximal
                      .motor()
                      .getTorque(proximal.motor().getCurrent(velocity.get(0, 0), u.get(0, 0)));
              var elbowTorque =
                  distal
                      .motor()
                      .getTorque(distal.motor().getCurrent(velocity.get(1, 0), u.get(1, 0)));
              var torque = VecBuilder.fill(shoulderTorque, elbowTorque);

              // Apply limits
              if (position.get(0, 0) < proximal.minAngle()) {
                position.set(0, 0, proximal.minAngle());
                if (velocity.get(0, 0) < 0.0) {
                  velocity.set(0, 0, 0.0);
                }
                if (torque.get(0, 0) < 0.0) {
                  torque.set(0, 0, 0.0);
                }
              }
              if (position.get(0, 0) > proximal.maxAngle()) {
                position.set(0, 0, proximal.maxAngle());
                if (velocity.get(0, 0) > 0.0) {
                  velocity.set(0, 0, 0.0);
                }
                if (torque.get(0, 0) > 0.0) {
                  torque.set(0, 0, 0.0);
                }
              }
              if (position.get(1, 0) < distal.minAngle()) {
                position.set(1, 0, distal.minAngle());
                if (velocity.get(1, 0) < 0.0) {
                  velocity.set(1, 0, 0.0);
                }
                if (torque.get(1, 0) < 0.0) {
                  torque.set(1, 0, 0.0);
                }
              }
              if (position.get(1, 0) > distal.maxAngle()) {
                position.set(1, 0, distal.maxAngle());
                if (velocity.get(1, 0) > 0.0) {
                  velocity.set(1, 0, 0.0);
                }
                if (torque.get(1, 0) > 0.0) {
                  torque.set(1, 0, 0.0);
                }
              }

              // Calculate acceleration
              var acceleration =
                  M(position)
                      .inv()
                      .times(
                          torque.minus(C(position, velocity).times(velocity)).minus(Tg(position)));

              // Return state vector
              return new MatBuilder<>(Nat.N4(), Nat.N1())
                  .fill(
                      velocity.get(0, 0),
                      velocity.get(1, 0),
                      acceleration.get(0, 0),
                      acceleration.get(1, 0));
            },
            state,
            voltage,
            dt));
  }

  private Matrix<N2, N2> M(Vector<N2> position) {
    var M = new Matrix<>(N2.instance, N2.instance);
    M.set(
        0,
        0,
        proximal.mass() * Math.pow(proximal.cgRadius(), 2.0)
            + distal.mass() * Math.pow(proximal.length(), 2.0)
            + proximal.moi());
    M.set(
        1,
        0,
        distal.mass() * proximal.length() * distal.cgRadius()
            * Math.cos(position.get(0, 0) - position.get(1,0)));
    M.set(0, 1, M.get(1,0));
    M.set(1, 1, distal.mass() * Math.pow(distal.cgRadius(), 2.0) + distal.moi());
    return M;
  }

  private Matrix<N2, N2> C(Vector<N2> position, Vector<N2> velocity) {
    var C = new Matrix<>(N2.instance, N2.instance);
    C.set(
        0,
        0,
        distal.mass()
            * proximal.length()
            * distal.cgRadius()
            * Math.sin(position.get(1, 0) - position.get(0,0))
            * velocity.get(1, 0));
    C.set(
        1,
        0,
        distal.mass()
            * proximal.length()
            * distal.cgRadius()
            * Math.sin(position.get(1, 0) - position.get(0,0))
            * (velocity.get(1, 0) - velocity.get(0,0)));
    C.set(0,1,C.get(1,0));
    C.set(
      1,
      1,
      distal.mass()
          * proximal.length()
          * distal.cgRadius()
          * Math.sin(position.get(1, 0) - position.get(0,0))
          * velocity.get(0, 0));            
    return C;
  }

  public Matrix<N2, N1> Tg(Vector<N2> position) {
    var Tg = new Matrix<>(N2.instance, N1.instance);
    Tg.set(
        0,
        0,
        (proximal.mass() * proximal.cgRadius() + distal.mass() * proximal.length())
                * g
                * Math.cos(position.get(0, 0))
        +distal.mass() * distal.cgRadius() * g * Math.cos(position.get(1,0)));
    Tg.set(
        1,
        0,
        distal.mass() * distal.cgRadius() * g * Math.cos(position.get(1, 0)));
    return Tg;
  }
}
