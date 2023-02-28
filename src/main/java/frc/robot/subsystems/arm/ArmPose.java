package frc.robot.subsystems.arm;

import java.util.Optional;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import frc.robot.subsystems.arm.json.ArmPresetsJson;

/** Represents a target position for the arm. */
public record ArmPose(
  Translation2d endEffectorPosition,
  double shoulderAngleRad,
  double elbowAngleRad) {

  // Preset arm position are read from arm_preset_poses.json

  public enum Preset {
    DEFENSE(0, null),
    INTAKE(0, null),
    DOUBLE_SUBSTATION(0, null),
    SCORE_HYBRID(0, null),
    SCORE_MID_CUBE(0, null),
    SCORE_HIGH_CUBE(0, null),
    SCORE_MID_CONE(0, null),
    SCORE_HIGH_CONE(0, null);

    private ArmPose pose;
    private int fileIdx;

    private Preset(int fileIdx, ArmPose pose) {
      this.fileIdx = fileIdx;
      this.pose = pose;
    }

    public ArmPose getPose() {
      return pose;
    }

    public double getX() {
      return pose.endEffectorPosition.getX();
    }

    public double getZ() {
      return pose.endEffectorPosition.getY();
    }

    public double getShoulderAngleRad() {
      return pose.shoulderAngleRad;
    }

    public double getElbowAngleRad() {
      return pose.elbowAngleRad;
    }

    public int getFileIdx() {
      return fileIdx;
    }


    public static void writePresets(ArmPresetsJson jsonPresets, ArmKinematics kinematics) {
      // write JSON file contents to ArmPose.Presets record

      Translation2d xz;
      Optional<Vector<N2>> theta;
      double shoulderAngleRad, elbowAngleRad;

      xz = new Translation2d(jsonPresets.defense().getX(), jsonPresets.defense().getY());
      theta = kinematics.inverse(xz.getX(), xz.getY());
      shoulderAngleRad = theta.get().get(0,0);
      elbowAngleRad = theta.get().get(1,0);
      DEFENSE.fileIdx = jsonPresets.defense().getFileIdx();
      DEFENSE.pose = new ArmPose(xz, shoulderAngleRad, elbowAngleRad);

      xz = new Translation2d(jsonPresets.intake().getX(), jsonPresets.intake().getY());
      theta = kinematics.inverse(xz.getX(), xz.getY());
      shoulderAngleRad = theta.get().get(0,0);
      elbowAngleRad = theta.get().get(1,0);
      INTAKE.fileIdx = jsonPresets.intake().getFileIdx();
      INTAKE.pose = new ArmPose(xz, shoulderAngleRad, elbowAngleRad);

      xz = new Translation2d(jsonPresets.double_substation().getX(), jsonPresets.double_substation().getY());
      theta = kinematics.inverse(xz.getX(), xz.getY());
      shoulderAngleRad = theta.get().get(0,0);
      elbowAngleRad = theta.get().get(1,0);
      DOUBLE_SUBSTATION.fileIdx = jsonPresets.double_substation().getFileIdx();
      DOUBLE_SUBSTATION.pose = new ArmPose(xz, shoulderAngleRad, elbowAngleRad);

      xz = new Translation2d(jsonPresets.score_hybrid().getX(), jsonPresets.score_hybrid().getY());
      theta = kinematics.inverse(xz.getX(), xz.getY());
      shoulderAngleRad = theta.get().get(0,0);
      elbowAngleRad = theta.get().get(1,0);
      SCORE_HYBRID.fileIdx = jsonPresets.score_hybrid().getFileIdx();
      SCORE_HYBRID.pose = new ArmPose(xz, shoulderAngleRad, elbowAngleRad);

      xz = new Translation2d(jsonPresets.score_mid_cube().getX(), jsonPresets.score_mid_cube().getY());
      theta = kinematics.inverse(xz.getX(), xz.getY());
      shoulderAngleRad = theta.get().get(0,0);
      elbowAngleRad = theta.get().get(1,0);
      SCORE_MID_CUBE.fileIdx = jsonPresets.score_mid_cube().getFileIdx();
      SCORE_MID_CUBE.pose = new ArmPose(xz, shoulderAngleRad, elbowAngleRad);

      xz = new Translation2d(jsonPresets.score_high_cube().getX(), jsonPresets.score_high_cube().getY());
      theta = kinematics.inverse(xz.getX(), xz.getY());
      shoulderAngleRad = theta.get().get(0,0);
      elbowAngleRad = theta.get().get(1,0);
      SCORE_HIGH_CUBE.fileIdx = jsonPresets.score_high_cube().getFileIdx();
      SCORE_HIGH_CUBE.pose = new ArmPose(xz, shoulderAngleRad, elbowAngleRad);

      xz = new Translation2d(jsonPresets.score_mid_cone().getX(), jsonPresets.score_mid_cone().getY());
      theta = kinematics.inverse(xz.getX(), xz.getY());
      shoulderAngleRad = theta.get().get(0,0);
      elbowAngleRad = theta.get().get(1,0);
      SCORE_MID_CONE.fileIdx = jsonPresets.score_mid_cone().getFileIdx();
      SCORE_MID_CONE.pose = new ArmPose(xz, shoulderAngleRad, elbowAngleRad);

      xz = new Translation2d(jsonPresets.score_high_cone().getX(), jsonPresets.score_high_cone().getY());
      theta = kinematics.inverse(xz.getX(), xz.getY());
      shoulderAngleRad = theta.get().get(0,0);
      elbowAngleRad = theta.get().get(1,0);
      SCORE_HIGH_CONE.fileIdx = jsonPresets.score_high_cone().getFileIdx();
      SCORE_HIGH_CONE.pose = new ArmPose(xz, shoulderAngleRad, elbowAngleRad);

    }
  }
}
