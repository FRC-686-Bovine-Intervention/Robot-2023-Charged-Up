package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.arm.json.ArmPresetsJson;

/** Represents a target position for the arm. */
public record ArmPose(Translation2d endEffectorPosition) {

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

    public double getY() {
      return pose.endEffectorPosition.getY();
    }

    public int getFileIdx() {
      return fileIdx;
    }


    public static void writePresets(ArmPresetsJson jsonPresets) {
      // write JSON file contents to ArmPose.Presets record
      DEFENSE.fileIdx = jsonPresets.defense().getFileIdx();
      DEFENSE.pose = new ArmPose(new Translation2d(jsonPresets.defense().getX(), jsonPresets.defense().getY()));

      INTAKE.fileIdx = jsonPresets.intake().getFileIdx();
      INTAKE.pose = new ArmPose(new Translation2d(jsonPresets.intake().getX(), jsonPresets.intake().getY()));

      DOUBLE_SUBSTATION.fileIdx = jsonPresets.double_substation().getFileIdx();
      DOUBLE_SUBSTATION.pose = new ArmPose(new Translation2d(jsonPresets.double_substation().getX(),jsonPresets.double_substation().getY()));

      SCORE_HYBRID.fileIdx = jsonPresets.score_hybrid().getFileIdx();
      SCORE_HYBRID.pose = new ArmPose(new Translation2d(jsonPresets.score_hybrid().getX(), jsonPresets.score_hybrid().getY()));

      SCORE_MID_CUBE.fileIdx = jsonPresets.score_mid_cube().getFileIdx();
      SCORE_MID_CUBE.pose = new ArmPose(new Translation2d(jsonPresets.score_mid_cube().getX(), jsonPresets.score_mid_cube().getY()));

      SCORE_HIGH_CUBE.fileIdx = jsonPresets.score_high_cube().getFileIdx();
      SCORE_HIGH_CUBE.pose = new ArmPose(new Translation2d(jsonPresets.score_high_cube().getX(), jsonPresets.score_high_cube().getY()));

      SCORE_MID_CONE.fileIdx = jsonPresets.score_mid_cone().getFileIdx();
      SCORE_MID_CONE.pose = new ArmPose(new Translation2d(jsonPresets.score_mid_cone().getX(), jsonPresets.score_mid_cone().getY()));

      SCORE_HIGH_CONE.fileIdx = jsonPresets.score_high_cone().getFileIdx();
      SCORE_HIGH_CONE.pose = new ArmPose(new Translation2d(jsonPresets.score_high_cone().getX(), jsonPresets.score_high_cone().getY()));

    }
  }
}
