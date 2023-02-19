// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

/**
 * Converts between joint angles and the end effector position.
 */
public class ArmKinematics {
  private final Translation2d shoulder;  // location of shoulder joint relative to center of turret rotation at floor height
  private final double l1;          // proximal arm length
  private final double l2;          // distal arm length
  private final double minTheta1;   // min allowable shoulder angle
  private final double maxTheta1;   // max allowable shoulder angle
  private final double minTheta21;  // min allowable elbow angle, relative to shoulder angle
  private final double maxTheta21;  // max allowable elbow angle, relative to shoulder angle

  
  public ArmKinematics(Translation2d shoulder, double l1, double l2, double minTheta1, double maxTheta1,
      double minTheta21, double maxTheta21) {
    this.shoulder = shoulder;
    this.l1 = l1;
    this.l2 = l2;
    this.minTheta1 = minTheta1;
    this.maxTheta1 = maxTheta1;
    this.minTheta21 = minTheta21;
    this.maxTheta21 = maxTheta21;
  }

  /** Converts joint angles to the end effector position. */
  public Translation2d forward(Vector<N2> theta) {
    double theta1  = theta.get(0,0);  // shoulder angle, relative to horizontal
    double theta21 = theta.get(1,0);  // elbow angle, relative to shoulder angle
    double theta2  = theta1 + theta21;         // elbow angle, relative to horizontal

    return shoulder.plus( new Translation2d(l1 * Math.cos(theta1) + l2 * Math.cos(theta2),
                                            l1 * Math.sin(theta1) + l2 * Math.sin(theta2)));
  }


  /** Converts the end effector position to joint angles. */
  public Optional<Vector<N2>> inverse(Translation2d position) {

    /* find where pivot point between 2 joints could be
     * (finding intersection of 2 circles centered at the base pivot and [x,z]
     * https://math.stackexchange.com/questions/256100/how-can-i-find-the-points-at-which-two-circles-intersect */

    // shoulder position
    double x0 = shoulder.getX();
    double y0 = shoulder.getY();

    // end effector position
    double x2 = position.getX();
    double y2 = position.getY();

    double D = Math.sqrt((x2-x0)*(x2-x0) + (y2-y0)*(y2-y0));
    double J = (l1*l1 - l2*l2) / (2.0*D*D);
    double term = (l1*l1 - l2*l2)/(D*D);
    double arg = 2*(l1*l1 + l2*l2)/(D*D) - term*term - 1.0;

    if (arg < 0) {
      // circles do not intersect (l1+l2 < D)
      return Optional.empty();
    }

    double K = Math.sqrt(arg) / 2.0;

    // two solutions of where circles will intersect
    double x1_a = (x0+x2)/2 + J*(x2-x0) +  K*(y2-y0);
    double y1_a = (y0+y2)/2 + J*(y2-y0) +  K*(x0-x2);

    double x1_b = (x0+x2)/2 + J*(x2-x0) -  K*(y2-y0);
    double y1_b = (y0+y2)/2 + J*(y2-y0) -  K*(x0-x2);
    
    double theta1_a = Math.atan2(y1_a-y0, x1_a-x0);
    double theta1_b = Math.atan2(y1_b-y0, x1_b-x0);

    // we select the angle with the elbow back (lowest theta1)
    double x1, y1, theta1;
    if (theta1_a < theta1_b) {
      x1 = x1_a;
      y1 = y1_a;
      theta1 = theta1_a;
    } else {
      x1 = x1_b;
      y1 = y1_b;
      theta1 = theta1_b;
    }

    double theta2 = Math.atan2(y2-y1, x2-x1);
    double theta21 = theta2 - theta1;

    // Exit if outside valid ranges for the joints
    if (theta1 < minTheta1 || theta1 > maxTheta1 ||
        theta21 < minTheta21 || theta21 > maxTheta21) {
      return Optional.empty();
    }

    return Optional.of(VecBuilder.fill(theta1, theta21));
  }
}
